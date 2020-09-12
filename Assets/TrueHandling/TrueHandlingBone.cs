using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * PhysicsBoneManager is created on each bone of each hand
 */

public class HandContact
{
    public Vector3 objectUnscaledLocalPos; //point on capsule axis
    public Vector3 objectSurfaceUnscaledLocalPos; //actual point on surface
    public Vector3 targetWorldPos;
    public Quaternion boneRotationDelta;
    public Vector3 objectLocalNormal;

    public int h;
    public int b;
    public int p; //capsule part id

    public Collider otherCollider;

    public float capsuleAxisFraction;

    public bool IsConnected(HandContact other)
    {
        return Vector3.Distance(targetWorldPos, other.targetWorldPos) <= Vector3.Distance(objectUnscaledLocalPos, other.objectUnscaledLocalPos);
    }
}

public class PhysicsBoneManager
{
    public Vector3 lastPos;
    public Vector3 thisPos;
    public Vector3 deltaPos;

    public Quaternion lastRot;
    public Quaternion thisRot;
    public Quaternion deltaRot;

    GameObject[] objects;
    public GameObject[] capsules;
    int h;
    int b;

    //TODO: make this not kinematic but just constrained, so it doesn't make objects go flying when pinched against world
    public GameObject kinematicRoot;

    public Dictionary<GameObject, HandContact[]> contactProposals;
    public Dictionary<GameObject, List<HandContact>> contactsAccepted;

    Dictionary<Collider, HandContact>[] contactPersistence;


    public TrueHandlingPhysics main;

    private HashSet<Collider>[] noCollided;

    public PhysicsBoneManager(TrueHandlingPhysics main_, HandState state, int bone_)
    {
        main = main_;
        h = state.right ? 1 : 0;
        b = bone_;

        Debug.Assert(state.active);

        contactProposals = new Dictionary<GameObject, HandContact[]>();
        contactPersistence = new Dictionary<Collider, HandContact>[HandState.BONE_CAPSULEPART_COUNT[b]];
        noCollided = new HashSet<Collider>[HandState.BONE_CAPSULEPART_COUNT[b]];

        for (int p=0; p< HandState.BONE_CAPSULEPART_COUNT[b]; p++)
        {
            contactPersistence[p] = new Dictionary<Collider, HandContact>();
            noCollided[p] = new HashSet<Collider>();
        }

        thisPos = state.GetBonePosition(b);
        thisRot = state.GetBoneAngle(b);

        objects = new GameObject[1];
        capsules = new GameObject[HandState.BONE_CAPSULEPART_COUNT[b]];

        kinematicRoot = objects[0] = new GameObject("tr");
        kinematicRoot.transform.parent = main.transform;
        kinematicRoot.layer = TrueHandlingPhysics.handCollisionLayer;

        Rigidbody rigid = kinematicRoot.AddComponent<Rigidbody>();
        rigid.isKinematic = true;
        //rigid.collisionDetectionMode = CollisionDetectionMode.Continuous;

        Transform capsuleOwner = kinematicRoot.transform;
        for (int i = 0; i < capsules.Length; i++)
        {
            // THESE DON'T IGNORE COLLISION IN THE JOINT
            GameObject cap = capsules[i] = new GameObject(i.ToString());
            cap.layer = TrueHandlingPhysics.handCollisionLayer;
            cap.AddComponent<CapsuleCollider>().direction = 2;
            cap.transform.parent = capsuleOwner;
        }
    }

    public void ProposeContacts(HandState state)
    {
        lastPos = thisPos;
        thisPos = state.GetBonePosition(b);
        deltaPos = thisPos - lastPos;

        lastRot = thisRot;
        thisRot = state.GetBoneAngle(b);
        deltaRot = Quaternion.Inverse(lastRot) * thisRot;

        contactsAccepted = new Dictionary<GameObject, List<HandContact>>();

        for (int c = 0; c < HandState.BONE_CAPSULEPART_COUNT[b]; c++)
        {
            CapsulePart part = state.boneCapsules[b][c];
            capsules[c].transform.localPosition = part.boneOffsetPos;
            capsules[c].transform.localRotation = part.boneOffsetAng;
            CapsuleCollider capc = capsules[c].GetComponent<CapsuleCollider>();
            capc.height = part.length + (part.radius - TrueHandlingPhysics.kinematicBoneRetraction) * 2.0f;
            capc.radius = part.radius - TrueHandlingPhysics.kinematicBoneRetraction;
        }

        kinematicRoot.GetComponent<Rigidbody>().MovePosition(lastPos);
        kinematicRoot.GetComponent<Rigidbody>().MoveRotation(lastRot);

        main.boneMaterialOverride[h, b] = null;

        contactProposals = new Dictionary<GameObject, HandContact[]>();

        for (int p = 0; p < HandState.BONE_CAPSULEPART_COUNT[b]; p++)
        {
            Dictionary<Collider, HandContact> lastPersistence = contactPersistence[p];
            Dictionary<Collider, HandContact> nextPersistence = new Dictionary<Collider, HandContact>();

            // USING CURRENT RADIUS/LENGTH/OFFSETPOS/OFFSETANG INSTEAD OF LAST, MIGHT BE WRONG BY A TINY BIT
            CapsulePart part = state.boneCapsules[b][p];

            Vector3 lastCapsulePos = lastPos + (lastRot * part.boneOffsetPos);
            Quaternion lastCapsuleRot = lastRot * part.boneOffsetAng;

            Vector3 thisCapsulePos = thisPos + (thisRot * part.boneOffsetPos);
            Quaternion thisCapsuleRot = thisRot * part.boneOffsetAng;

            Vector3 lastOffset = lastCapsuleRot * (Vector3.forward * part.length * 0.5f);
            
            int hits = Physics.OverlapCapsuleNonAlloc(lastCapsulePos + lastOffset, lastCapsulePos - lastOffset, part.radius, main.triggerBuffer, main.handCollisionLayerCollideMask);

            for (int i = 0; i < hits; i++)
            {
                Collider other = main.triggerBuffer[i];
                if (other.attachedRigidbody == null) { continue; }
                GameObject obj = other.attachedRigidbody == null ? other.gameObject : other.attachedRigidbody.gameObject;

                HandContact contact;

                if (lastPersistence.ContainsKey(other))
                {
                    //pen = CapsulePenetration.TestFixedNormal(lastCapsulePos, lastCapsuleRot, part.length, part.radius, other, other.transform.position, other.transform.rotation, other.transform.rotation * lastPersistence[other].objectLocalNormal);
                    contact = lastPersistence[other];
                } else {
                    CapsulePenetrationData pen = CapsulePenetration.Test(lastCapsulePos, lastCapsuleRot, part.length, part.radius, other, other.transform.position, other.transform.rotation);

                    if (pen == null)
                    {
                        continue;
                    }

                    // If the contact point is inside another capsule, skip it
                    // This is only running to prevent the contact from being created, but the finger can bend/object can move such that
                    // an existing contact violates this. To fix that is hard though, because the target point on the capsule surface
                    // can move when fingers rotate
                    bool pinched = false;
                    for (int b2 = 0; b2 < HandState.NUM_BONES; b2++) {
                        Vector3 bp = state.GetBonePosition(b2);
                        Quaternion ba = state.GetBoneAngle(b2);
                        for (int p2 = 0; p2 < HandState.BONE_CAPSULEPART_COUNT[b2]; p2++)
                        {
                            if (b2 == b && p2 == p) {
                               continue;
                            }
                            CapsulePart otherPart = state.boneCapsules[b2][p2];
                            Vector3 cp = bp + (ba * otherPart.boneOffsetPos);
                            Quaternion ca = ba * otherPart.boneOffsetAng;

                            Vector3 localHit = Quaternion.Inverse(ca) * (pen.hitPos - cp);

                            // Intersection with the endcap doesn't count, so you can grab with outside of bent fingers
                            if (localHit.z < -otherPart.length*0.5f || localHit.z > otherPart.length*0.5f)
                            {
                                continue;
                            }

                            Vector3 axPos = new Vector3(0,0, localHit.z);

                            if (Vector3.Distance(localHit, axPos) < otherPart.radius)
                            {
                                pinched = true;
                                break;
                            }
                        }
                        if (pinched)
                        {
                            break;
                        }
                    }
                    if (pinched)
                    {
                        continue;
                    }
                    
                    contact = new HandContact();
                    contact.capsuleAxisFraction = pen.capsuleHitFraction;
                    contact.objectLocalNormal = Quaternion.Inverse(other.transform.rotation) * pen.hitNorm;
                    contact.objectSurfaceUnscaledLocalPos = Quaternion.Inverse(other.transform.rotation) * (pen.hitPos - other.transform.position);
                    contact.objectUnscaledLocalPos = contact.objectSurfaceUnscaledLocalPos + contact.objectLocalNormal * part.radius;
                    contact.h = h;
                    contact.b = b;
                    contact.p = p;
                    contact.otherCollider = other;
                }

                //project the capsule pos onto the capsule axis
                Vector3 capsuleLocalHitPos = new Vector3(0f, 0f, (contact.capsuleAxisFraction - 0.5f) * part.length);
                contact.targetWorldPos = thisCapsulePos + (thisCapsuleRot * capsuleLocalHitPos);
                contact.boneRotationDelta = deltaRot;

                nextPersistence[other] = contact;

                //Gizmoz.DrawLine(pen.hitPos, pen.hitPos - pen.capsuleDepenetrationTranslation, Color.red);
                //main.boneMaterialOverride[h, b] = main.touchingMaterial;

                HandContact[] contacts;

                if (contactProposals.ContainsKey(obj))
                {
                    HandContact[] oldContacts = contactProposals[obj];
                    contacts = new HandContact[oldContacts.Length + 1];
                    for (int i2 = 0; i2 < oldContacts.Length; i2++)
                    {
                        contacts[i2] = oldContacts[i2];
                    }
                }
                else
                {
                    contacts = new HandContact[1];
                }

                contacts[contacts.Length - 1] = contact;
                contactProposals[obj] = contacts;
            }

            contactPersistence[p] = nextPersistence;
        }
    }

    public void UpdateNoCollides(HandState state, Dictionary<GameObject, Vector3> estimatedObjectPos, Dictionary<GameObject, Quaternion> estimatedObjectAng)
    {
        foreach(var kv in estimatedObjectPos)
        {
            GameObject obj = kv.Key;
            if (contactProposals.ContainsKey(obj))
            {
                foreach (HandContact contact in contactProposals[obj])
                {
                    if (!noCollided[contact.p].Contains(contact.otherCollider))
                    {
                        noCollided[contact.p].Add(contact.otherCollider);
                        Physics.IgnoreCollision(capsules[contact.p].GetComponent<CapsuleCollider>(), contact.otherCollider, true);
                    }
                }
            }
        }

        for (int p = 0; p<noCollided.Length; p++)
        {
            CapsuleCollider cap = capsules[p].GetComponent<CapsuleCollider>();
            LinkedList<Collider> toReCollide = new LinkedList<Collider>();
            foreach (Collider collider in noCollided[p])
            {
                GameObject obj = collider.attachedRigidbody.gameObject;

                //Gizmoz.DrawLine(cap.transform.TransformPoint(Vector3.zero), collider.transform.TransformPoint(Vector3.zero), Color.green);

                if (estimatedObjectPos.ContainsKey(obj))
                {
                    continue;
                }

                Vector3 dir;
                float dist;

                if (!Physics.ComputePenetration(cap, cap.transform.position, cap.transform.rotation, collider, collider.transform.position, collider.transform.rotation, out dir, out dist)) {
                    toReCollide.AddLast(collider);
                }
            }
            foreach (Collider collider in toReCollide)
            {
                if (noCollided[p].Contains(collider))
                {
                    noCollided[p].Remove(collider);
                    Physics.IgnoreCollision(cap, collider, false);
                }   
            }
        }
    }

    public void Cleanup()
    {
        for (int i = 0; i < objects.Length; i++)
        {
            Object.Destroy(objects[i]);
        }
    }
}
