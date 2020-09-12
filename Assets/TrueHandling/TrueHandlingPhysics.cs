using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Main TrueHandling physics logic
 */

public class TrueHandlingPhysics : MonoBehaviour {

    public const float kinematicBoneRetraction = 0.002f;
    public const int handCollisionLayer = 22;
    [HideInInspector]
    public int handCollisionLayerCollideMask; // calculated in Start

    public GameObject sourceGameObject;
    public HandStateSource source; // it's an interface so not serializable

    public Material touchingMaterial;
    //public Material stickyMaterial;

    [HideInInspector]
    public Material[,] boneMaterialOverride;

    private PhysicsBoneManager[,] boneManager;

    [HideInInspector]
    public Collider[] triggerBuffer;
    [HideInInspector]
    public RaycastHit[] raycastBuffer;

    public Dictionary<GameObject, GameObject> objectCarriers;
    //public Dictionary<GameObject, GameObject> objectCarrierProxies;

    List<KeyValuePair<CapsuleCollider, Collider>> noCollided;

    void Start () {
        //configure collision matrix: no intercollision and otherwise inherit default
        handCollisionLayerCollideMask = 0;
        for (int i=0; i<32; i++)
        {
            if (i == handCollisionLayer)
            {
                Physics.IgnoreLayerCollision(handCollisionLayer, i, true);
            } else
            {
                if (Physics.GetIgnoreLayerCollision(0, i))
                {
                    Physics.IgnoreLayerCollision(handCollisionLayer, i, true);
                } else
                {
                    Physics.IgnoreLayerCollision(handCollisionLayer, i, false);
                    handCollisionLayerCollideMask |= 1 << i;
                }
            }
        }

        HandStateSource[] sources = sourceGameObject.GetComponents<HandStateSource>();
        Debug.Assert(sources.Length == 1);
        source = sources[0];

        source.FixedUpdateDone += FixedUpdatePropagation;

        triggerBuffer = new Collider[32];
        raycastBuffer = new RaycastHit[32];

        boneMaterialOverride = new Material[2, HandState.NUM_BONES];
        boneManager = new PhysicsBoneManager[2, HandState.NUM_BONES];

        objectCarriers = new Dictionary<GameObject, GameObject>();
        //objectCarrierProxies = new Dictionary<GameObject, GameObject>();

        noCollided = new List<KeyValuePair<CapsuleCollider, Collider>>();
    }

    void FixedUpdatePropagation(object sender, object noArgs) {

        HandState[] states = { source.GetHandState(false), source.GetHandState(true) };

        Dictionary<GameObject, List<HandContact>> objContacts = new Dictionary<GameObject, List<HandContact>>();

        for (int h = 0; h < 2; h++)
        {
            HandState state = states[h];

            if (state.active)
            {
                if (boneManager[h,0] == null)
                {
                    for (int b = 0; b < HandState.NUM_BONES; b++)
                    {
                        boneManager[h, b] = new PhysicsBoneManager(this, state, b);
                    }
                }

                for (int b = 0; b < HandState.NUM_BONES; b++)
                {
                    boneManager[h, b].ProposeContacts(state);

                    //add contacts to dataset to prune stretched ones
                    foreach (var kv in boneManager[h, b].contactProposals)
                    {
                        GameObject obj = kv.Key;
                        if (!objContacts.ContainsKey(obj))
                        {
                            objContacts[obj] = new List<HandContact>();
                        }                        
                        foreach (HandContact contact in kv.Value)
                        {
                            objContacts[obj].Add(contact);
                        }
                    }
                }

            } else
            {
                if (boneManager[h, 0] != null)
                {
                    for (int b = 0; b < HandState.NUM_BONES; b++)
                    {
                        boneManager[h, b].Cleanup();
                        boneManager[h, b] = null;
                    }
                }
            }

        }

        Dictionary<GameObject, HashSet<HandContact>> filteredHandContacts = new Dictionary<GameObject, HashSet<HandContact>>();

        // Find contact pairs that have "stretched" meaning the bones are farther apart than when they touched the object, and cluster them.
        // TODO: Instead of this, break connections when the contact point on the finger is "above" the contact point on the object
        // based on the normal. that way you can pick up concave objects from the inside
        foreach (var kv in objContacts)
        {
            GameObject obj = kv.Key;
            HandContact[] contacts = kv.Value.ToArray();
            List<HashSet<HandContact>> connectedComponents = new List<HashSet<HandContact>>();

            for (int a = 0; a < contacts.Length; a++)
            {
                HashSet<HandContact> ourSet = new HashSet<HandContact>();
                ourSet.Add(contacts[a]);
                connectedComponents.Add(ourSet);
            }

            for (int a = 0; a < contacts.Length - 1; a++)
            {
                HandContact first = contacts[a];
                HashSet<HandContact> ourSet = null;

                foreach (HashSet<HandContact> comp in connectedComponents)
                {
                    if (comp.Contains(first))
                    {
                        ourSet = comp;
                    }
                }
                Debug.Assert(ourSet != null);

                for (int b = a + 1; b < contacts.Length; b++)
                {
                    HandContact second = contacts[b];
                    HashSet<HandContact> theirSet = null;

                    foreach (HashSet<HandContact> comp in connectedComponents)
                    {
                        if (comp.Contains(second))
                        {
                            theirSet = comp;
                        }
                    }
                    if (ourSet == theirSet)
                    {
                        continue;
                    }

                    Debug.Assert(theirSet != null);

                    if (first.IsConnected(second))
                    {
                        ourSet.UnionWith(theirSet);
                        connectedComponents.Remove(theirSet);
                    }
                }
            }

            List<HashSet<HandContact>> fullyConstrainedConnectedComponents = new List<HashSet<HandContact>>();

            foreach (HashSet<HandContact> component in connectedComponents)
            {
                HandContact[] componentArray = new HandContact[component.Count];
                component.CopyTo(componentArray);
                bool fullyConstrained = false;

                // Decides if object is fully constrained ("held"), TODO try globally reasoning about all contacts
                for (int a = 0; a<componentArray.Length-1; a++)
                {
                    for (int b = a+1; b<componentArray.Length; b++)
                    {
                        HandContact ca = componentArray[a];
                        HandContact cb = componentArray[b];

                        if (ca.h == cb.h) {
                            if (ca.b == cb.b || HandState.POINT_PARENT[ca.b] == cb.b || HandState.POINT_PARENT[cb.b] == ca.b)
                            {
                                continue;
                            }
                        }

                        if (Vector3.Dot(ca.objectLocalNormal, cb.objectLocalNormal) < 0.01)
                        {
                            // Skip points where the object could easily rotate out of grip
                            Vector3 offset = Vector3.Normalize(ca.objectSurfaceUnscaledLocalPos - cb.objectSurfaceUnscaledLocalPos);
                            float thresh = 0.5f;
                            if (Mathf.Abs(Vector3.Dot(ca.objectLocalNormal, offset)) < thresh || Mathf.Abs(Vector3.Dot(cb.objectLocalNormal, offset)) < thresh)
                            {
                                continue;
                            }

                            fullyConstrained = true;
                        }
                    }
                }

                // Don't hold objects unless one of the bones is a fingertip, TODO make the system better so this isn't necessary
                bool hasFingertip = false;
                for (int a = 0; a < componentArray.Length; a++)
                {
                    //hasFingertip |= componentArray[a].b == 2; //second from thumbtip
                    hasFingertip |= componentArray[a].b == 3;
                    hasFingertip |= componentArray[a].b == 7;
                    hasFingertip |= componentArray[a].b == 11;
                    //hasFingertip |= componentArray[a].b == 15;
                    //hasFingertip |= componentArray[a].b == 19;
                }


                if (fullyConstrained && hasFingertip)
                {
                    fullyConstrainedConnectedComponents.Add(component);
                }
            }

            if (fullyConstrainedConnectedComponents.Count > 0)
            {
                //TODO: prioritize better
                fullyConstrainedConnectedComponents.Sort((x, y) => y.Count.CompareTo(x.Count));
                filteredHandContacts[obj] = fullyConstrainedConnectedComponents[0];
            }

        }


        Dictionary<GameObject, Vector3> movedObjectPos = new Dictionary<GameObject, Vector3>();
        Dictionary<GameObject, Quaternion> movedObjectAng = new Dictionary<GameObject, Quaternion>();

        foreach (var kv in filteredHandContacts)
        {
            GameObject obj = kv.Key;
            Rigidbody rigid = obj.GetComponent<Rigidbody>();
            if (rigid == null)
            {
                continue;
            }

            HashSet<HandContact> contacts = kv.Value;

            List<ObjectMotionSolutionUnit> solveUnits = new List<ObjectMotionSolutionUnit>();
            Vector3 objectLocalAxisPositionCentroid = Vector3.zero;
            Vector3 targetWorldAxisPositionCentroid = Vector3.zero;

            foreach (HandContact contact in contacts) {
                ObjectMotionSolutionUnit piece = new ObjectMotionSolutionUnit(contact, obj.transform.rotation); // states[h].boneCapsules[b][contact.capsuleIndex], bonePos, boneAng, oldBoneAng, obj.transform);
                solveUnits.Add(piece);
                objectLocalAxisPositionCentroid += piece.objectLocalAxisPosition;
                targetWorldAxisPositionCentroid += piece.targetWorldAxisPosition;

                //Vector3 p1 = obj.transform.position + obj.transform.rotation * contact.objectUnscaledLocalPos;
                //Gizmoz.DrawLine(p1, p1 + obj.transform.TransformDirection(contact.objectLocalNormal)*0.01f, Color.green);
                //Gizmoz.DrawLine(p1, piece.targetWorldAxisPosition, Color.white);

                //Vector3 p2 = obj.transform.position + obj.transform.rotation * contact.objectSurfaceUnscaledLocalPos;
                //Gizmoz.DrawLine(p2, piece.targetWorldAxisPosition, Color.cyan);
            }

            objectLocalAxisPositionCentroid /= solveUnits.Count;
            targetWorldAxisPositionCentroid /= solveUnits.Count;

            //world axis position + 4 for tetrahetron for rotation alignment
            int l = solveUnits.Count * 5;

            double[,] A = new double[3, l];
            double[,] B = new double[3, l];

            int cur = 0;
            float mainScale = 20f;
            foreach (ObjectMotionSolutionUnit unit in solveUnits)
            {
                FillMatrixColumn(A, cur, mainScale * (unit.objectLocalAxisPosition - objectLocalAxisPositionCentroid));
                FillMatrixColumn(B, cur, mainScale * (unit.targetWorldAxisPosition - targetWorldAxisPositionCentroid));
                cur++;
                for (int t = 0; t < 4; t++)
                {
                    FillMatrixColumn(A, cur, unit.GetFromTetrahedron(t));
                    FillMatrixColumn(B, cur, unit.GetToTetrahedron(t));
                    cur++;
                }
            }

            Debug.Assert(cur == l);

            double[,] BAT = new double[3, 3];
            alglib.rmatrixgemm(3, 3, l, 1, B, 0, 0, 0, A, 0, 0, 1, 0, ref BAT, 0, 0);

            double[] W = new double[3];
            double[,] U = new double[3, 3];
            double[,] VT = new double[3, 3];

            alglib.rmatrixsvd(BAT, 3, 3, 2, 2, 2, out W, out U, out VT);

            double[,] UVT = new double[3, 3];
            alglib.rmatrixgemm(3, 3, 3, 1, U, 0, 0, 0, VT, 0, 0, 0, 0, ref UVT, 0, 0);

            double sqrtThing = 1 + UVT[0, 0] + UVT[1, 1] + UVT[2, 2];
            if (sqrtThing <= 0)
            {
                Debug.Log("matrix solution error");
                sqrtThing = 0.0001;
            }

            double w = System.Math.Sqrt(sqrtThing) / 2.0;
            double iw4 = 1.0 / (w * 4);

            Quaternion rotation = new Quaternion((float)((UVT[2, 1] - UVT[1, 2]) * iw4),
                (float)((UVT[0, 2] - UVT[2, 0]) * iw4),
                (float)((UVT[1, 0] - UVT[0, 1]) * iw4), (float)w);

            rotation.Normalize();

            Vector3 pos = targetWorldAxisPositionCentroid - rotation * objectLocalAxisPositionCentroid;

            movedObjectPos[obj] = pos;
            movedObjectAng[obj] = rotation;
        }
        
        //setup kinematic carriers
        LinkedList<GameObject> carriersToRemove = new LinkedList<GameObject>();
        foreach (var kv in objectCarriers)
        {
            if (!movedObjectPos.ContainsKey(kv.Key))
            {
                carriersToRemove.AddLast(kv.Key);
            }
        }
        foreach (GameObject obj in carriersToRemove)
        {
            objectCarriers[obj].GetComponent<FixedJoint>().connectedBody = null; //NECESSARY BECAUSE IT WONT DESTROY TIL AFTER THE PHYSICS UPDATE
            Destroy(objectCarriers[obj]);
            objectCarriers.Remove(obj);
            //if (objectCarrierProxies.ContainsKey(obj))
            //{
            //   Destroy(objectCarrierProxies[obj]);
            //  objectCarrierProxies.Remove(obj);
            // }
        }

        foreach (var kv in movedObjectPos)
        {
            GameObject obj = kv.Key;
            Vector3 pos = kv.Value;
            Quaternion ang = movedObjectAng[obj];

            if (!objectCarriers.ContainsKey(obj))
            {
                GameObject car = objectCarriers[obj] = new GameObject("car");
                car.transform.parent = transform;
                car.transform.position = obj.transform.position;
                car.transform.rotation = obj.transform.rotation;

                Rigidbody r = car.AddComponent<Rigidbody>();
                r.isKinematic = true;

                FixedJoint f = car.AddComponent<FixedJoint>();
                f.enablePreprocessing = false;
                f.autoConfigureConnectedAnchor = false;
                f.connectedBody = obj.GetComponent<Rigidbody>(); 
                f.anchor = Vector3.zero;
                f.connectedAnchor = Vector3.zero;
                
            }

            Rigidbody rig = objectCarriers[obj].GetComponent<Rigidbody>();
            rig.MovePosition(pos);
            rig.MoveRotation(ang);
        }

        // Was used for IK
        /*
        List<IKPointTarget> pointTargetsList = new List<IKPointTarget>();
        foreach (var objList in stretching)
        {
            GameObject obj = objList.Key;

            for (int h = 0; h < 2; h++)
            {
                if (!states[h].active)
                {
                    continue;
                }

                for (int b = 0; b < HandState.NUM_BONES; b++)
                {
                    Dictionary<GameObject, CapsuleContactPoint[]> proposal = boneManager[h, b].contactJointProposals;

                    //only ik constrain fingertips FINGERFILTER
                    if (b % 4 != 3 || b>7)
                    {
                        continue;
                    }

                    if (proposal.ContainsKey(objList.Key))
                    {
                        CapsuleContactPoint[] contacts = proposal[objList.Key];
                        Debug.Assert(contacts.Length > 0);

                        foreach (CapsuleContactPoint contact in contacts)
                        {
                            IKPointTarget next = new IKPointTarget();
                            next.h = h;
                            next.b = b;
                            next.localPos = states[h].boneCapsules[b][contact.capsuleIndex].AxisFractionToBonePosition(contact.axisFraction);
                            Vector3 unscaledLocal = obj.transform.InverseTransformDirection(obj.transform.TransformVector(contact.otherPos)) + contact.otherNormal* states[h].boneCapsules[b][contact.capsuleIndex].radius;
                            next.worldTargetPos = estimatedObjectPos[obj] + estimatedObjectAng[obj] * unscaledLocal;
                            pointTargetsList.Add(next);
                        }
                    }
                }
            }
        }
        IKGapTarget[] gapTargets = new IKGapTarget[0];
        IKPointTarget[] pointTargets = pointTargetsList.ToArray();
        FABRIK.Solve(states, pointTargets, gapTargets);
        */

        for (int h = 0; h < 2; h++)
        {
            HandState state = states[h];

            if (state.active)
            {
                for (int b = 0; b < HandState.NUM_BONES; b++)
                {
                    boneManager[h, b].UpdateNoCollides(state, movedObjectPos, movedObjectAng);
                }
            }   
        }
    }

    private void FillMatrixColumn(double[,] mat, int col, Vector3 with)
    {
        mat[0, col] = with.x;
        mat[1, col] = with.y;
        mat[2, col] = with.z;
    }

    private class ObjectMotionSolutionUnit
    {
        public Vector3 objectLocalAxisPosition;
        public Vector3 targetWorldAxisPosition;

        private Quaternion objectRotation;
        private Vector3 objectLocalNormal;

        public static float rotationAlignmentScale = 0.1f;
        public static readonly Vector3[] tetrahedron = {
                new Vector3(Mathf.Sqrt(8f / 9f), 0, -1f / 3f),
                new Vector3(-Mathf.Sqrt(2f / 9f), Mathf.Sqrt(2f / 3f), -1f / 3f),
                new Vector3(-Mathf.Sqrt(2f / 9f), -Mathf.Sqrt(2f / 3f), -1f / 3f),
                new Vector3(0, 0, 1),
            };

        public ObjectMotionSolutionUnit(HandContact ccp_, Quaternion currentAngle)
        {
            objectLocalAxisPosition = ccp_.objectUnscaledLocalPos;
            targetWorldAxisPosition = ccp_.targetWorldPos;
            objectRotation = ccp_.boneRotationDelta * currentAngle;
        }

        public Vector3 GetFromTetrahedron(int i)
        {
            Vector3 vec = tetrahedron[i] * rotationAlignmentScale;
            return vec;
        }

        public Vector3 GetToTetrahedron(int i)
        {
            return objectRotation * GetFromTetrahedron(i);
        }

    }

}

