using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Represents pose of one hand
 * 
 * Pointcloud order:
 * 0,1,2,3: thumb joints and tip
 * 4-7: index finger joints and tip
 * 8-19: other fingers
 * 20: wrist
 * 21: elbow
 * 
 * 
 * Bones:
 * each bone is between two pointcloud points, the bone id is equal to the outermost point id
 * the bone position is the midpoint of the two points
 * the bone rotation is along the axis between them with an upvector calculated from other stuff
 * there are 21 bones (id 0-20), the elbow doesn't have a bone (the arm is bone 20; the palm has 5 bones)
 *
 * note:
 * finger bone 0 is metacarpal (part of the palm)
 * finger bone 1 is the proximal phalanges, the metacarpals are just part of the palm in this even though they actually can move a little
 * finger bone 2 intermediate
 * finger bone 3 distal
 *
 * thumb bone 0 is the offset from wrist to metacarpal joint
 * thumb bone 1 is metacarpal, bone 2 proximal, bone 3 distal
 */

public class CapsulePart
{
    public float length;
    public float radius;
    public Vector3 boneOffsetPos;
    public Quaternion boneOffsetAng;
    public int idInHand;

    public CapsulePart()
    {
        boneOffsetPos = Vector3.zero;
        boneOffsetAng = Quaternion.identity;
        length = 0.01f;
        radius = 0.01f;
        idInHand = 0;
    }

    public float PartLocalPointToAxisFraction(Vector3 point)
    {
        return Mathf.Clamp((point.z / length) + 0.5f, 0, 1);
    }

    public Vector3 AxisFractionToBonePosition(float axisFraction)
    {
        Vector3 bp = new Vector3(0, 0, (axisFraction - 0.5f) * length);
        return boneOffsetPos + boneOffsetAng * bp;
    }
}

public class HandState
{
    public const int NUM_POINTS = 22;
    public const int NUM_BONES = 21;

    public const int NUM_CAPSULEPARTS = 21; //sync with below
    public static readonly int[] BONE_CAPSULEPART_COUNT = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

    public static readonly int[] POINT_PARENT = { 20, 0, 1, 2, 20, 4, 5, 6, 20, 8, 9, 10, 20, 12, 13, 14, 20, 16, 17, 18, 21, -1 };
    public static readonly HashSet<int>[] POINT_CHILDREN;

    public static readonly int[] POINT_KNUCKLE = { 0, 4, 8, 12, 16 };
    public static readonly int[] POINT_LOJOINT = { 1, 5, 9, 13, 17 };
    public static readonly int[] POINT_HIJOINT = { 2, 6, 10, 14, 18 };
    public static readonly int[] POINT_TIP = { 3, 7, 11, 15, 19 };
    public const int POINT_WRIST = 20;
    public const int POINT_ELBOW = 21;

    public static readonly int[] BONE_FINGER_INPALM = POINT_KNUCKLE;
    public static readonly int[] BONE_FINGER_LO = POINT_LOJOINT;
    public static readonly int[] BONE_FINGER_HI = POINT_HIJOINT;
    public static readonly int[] BONE_FINGER_TIP = POINT_TIP;
    public const int BONE_ARM = POINT_WRIST;

    static HandState()
    {
        POINT_CHILDREN = new HashSet<int>[NUM_POINTS];
        for (int p = 0; p < NUM_POINTS; p++)
        {
            POINT_CHILDREN[p] = new HashSet<int>();
        }
        for (int p = 0; p < NUM_POINTS; p++)
        {
            if (POINT_PARENT[p] != -1)
            {
                POINT_CHILDREN[POINT_PARENT[p]].Add(p);
            }
        }
    }

    public Vector3[] points;
    public float[] pointDistanceFromParent;

    public CapsulePart[][] boneCapsules;

    public bool active;
    public bool notLost;
    public bool right;
    public Vector3[] knuckleOffset;

    private Quaternion[] leapReportedBoneAngles;

    public HandState()
    {
        points = new Vector3[NUM_POINTS];
        pointDistanceFromParent = new float[NUM_POINTS];

        //for now just doing capsules along bones, but this can change
        boneCapsules = new CapsulePart[NUM_BONES][];
        int gc = 0;
        for (int b = 0; b<NUM_BONES; b++)
        {
            boneCapsules[b] = new CapsulePart[BONE_CAPSULEPART_COUNT[b]];
            for (int c=0; c<boneCapsules[b].Length; c++)
            {
                boneCapsules[b][c] = new CapsulePart();
                boneCapsules[b][c].idInHand = gc;
                gc++;
            }
        }
        knuckleOffset = new Vector3[5];

        leapReportedBoneAngles = new Quaternion[NUM_BONES];
    }


    public void UpdateFromLeapHand(Leap.Hand hand)
    {
        Debug.Assert(hand.IsLeft != right);

        Vector3 v = FixV(hand.Arm.Center);
        Quaternion q = FixQ(hand.Arm.Rotation);

        points[POINT_ELBOW] = v - (q * Vector3.forward * hand.Arm.Length * 0.5f);
        points[POINT_WRIST] = v + (q * Vector3.forward * hand.Arm.Length * 0.5f);
        boneCapsules[BONE_ARM][0].radius = hand.Arm.Width * 0.5f;

        leapReportedBoneAngles[BONE_ARM] = FixQ(hand.Arm.Rotation);

        for (int f = 0; f < 5; f++)
        {
            Leap.Bone lbone = hand.Fingers[f].bones[1];
            float rad = lbone.Width * 0.5f;

            points[POINT_KNUCKLE[f]] = FixV(lbone.PrevJoint);
            points[POINT_LOJOINT[f]] = FixV(lbone.NextJoint);

            lbone = hand.Fingers[f].bones[3];

            points[POINT_HIJOINT[f]] = FixV(lbone.PrevJoint);
            points[POINT_TIP[f]] = FixV(lbone.NextJoint);

            for (int b = 0; b<4; b++)
            {
                boneCapsules[f * 4 + b][0].radius = rad;
                leapReportedBoneAngles[f * 4 + b] = FixQ(hand.Fingers[f].bones[b].Rotation);
            }
        }

        for (int p = 0; p<NUM_BONES; p++)
        {
            boneCapsules[p][0].length = pointDistanceFromParent[p] = Vector3.Distance(points[p], points[POINT_PARENT[p]]);

            //joint retraction, might help with physics in some cases to not have capsules overlappin
            /*if (p < 20 && p % 4 > 0 && p % 4 < 3)
            {
                boneCapsules[p][0].length -= 0.006f;
            }*/
        }


        //cosmetic changes
        float pArmRadius = boneCapsules[BONE_ARM][0].radius;
        Vector3 pPalmForward = leapReportedBoneAngles[BONE_FINGER_INPALM[2]] * Vector3.forward;
        Vector3 pPalmFront = leapReportedBoneAngles[BONE_FINGER_INPALM[2]] * Vector3.down;
        Vector3 pSidewaysToThumb = leapReportedBoneAngles[BONE_FINGER_INPALM[2]] * Vector3.right * (right ? -1f : 1f);

        boneCapsules[BONE_ARM][0].radius -= 0.006f;

        SetPrincipalCapsulePoints(BONE_FINGER_INPALM[0], 
            points[POINT_WRIST] + pArmRadius * -0.4f * pSidewaysToThumb - pArmRadius * 0.2f * pPalmFront - 0.01f * pPalmForward, 
            points[POINT_KNUCKLE[0]] + pArmRadius * -0.05f * pPalmFront + 0.002f * pPalmForward - 0.002f * pSidewaysToThumb);
        SetPrincipalCapsulePoints(BONE_FINGER_INPALM[1],
            points[POINT_WRIST] + pArmRadius * 0.35f * pSidewaysToThumb + pArmRadius * 0.1f * pPalmFront,
            points[POINT_KNUCKLE[1]] - 0.002f * pSidewaysToThumb);
        SetPrincipalCapsulePoints(BONE_FINGER_INPALM[4],
            points[POINT_WRIST] + pArmRadius *-0.35f * pSidewaysToThumb + pArmRadius*0.2f*pPalmFront,
            points[POINT_KNUCKLE[4]] + 0.002f * pSidewaysToThumb);

        for (int f = 0; f < 5; f++)
        {
            boneCapsules[BONE_FINGER_INPALM[f]][0].radius += (f==0) ? 0.005f : ((f == 1 || f == 4) ? 0.0035f : 0.0025f);
        }
        boneCapsules[BONE_FINGER_LO[0]][0].radius += 0.0015f;

    }

    private void SetPrincipalCapsulePoints(int b, Vector3 p1, Vector3 p2)
    {
        Vector3 p1p2 = p2 - p1;
        Quaternion capsuleGlobalRot = PointQuaternion(leapReportedBoneAngles[b], p1p2);
        Vector3 offset = (0.5f * (p1 + p2)) - GetBonePosition(b);
        Quaternion boneAng = GetBoneAngle(b);

        boneCapsules[b][0].boneOffsetPos = Quaternion.Inverse(boneAng) * offset;
        boneCapsules[b][0].boneOffsetAng = Quaternion.Inverse(boneAng) * capsuleGlobalRot ;
        boneCapsules[b][0].length = p1p2.magnitude;
    }

    //make quaternion point towards vector with minimal twisting
    public static Quaternion PointQuaternion(Quaternion from, Vector3 point)
    {
        Vector3 fromForward = from * Vector3.forward;
        float ang = Vector3.Angle(fromForward, point);

        if (ang == 0f)
        {
            return from;
        }

        Quaternion offset = Quaternion.AngleAxis(ang, Vector3.Cross(fromForward, point));
        return offset * from;
    }

    public Vector3 GetBonePosition(int b)
    {
        return 0.5f * (points[b] + points[POINT_PARENT[b]]);
    }

    public Quaternion GetBoneAngle(int b)
    {
        return Quaternion.LookRotation(GetBoneForward(b), GetBoneUp(b));
    }

    public Vector3 GetBoneForward(int b)
    {
        return (points[b] - points[POINT_PARENT[b]]).normalized;
    }

    public Vector3 GetBoneUp(int b)
    {
        return PointQuaternion(leapReportedBoneAngles[b], GetBoneForward(b)) * Vector3.up;
    }
    
    public Vector3 LocalToWorld(int b, Vector3 pos)
    {
        return GetBonePosition(b) + GetBoneAngle(b) * pos;
    }

    // why is this necessary...
    private Vector3 FixV(Leap.Vector v)
    {
        return new Vector3(v.x, v.y, v.z);
    }

    private Quaternion FixQ(Leap.LeapQuaternion q)
    {
        return new Quaternion(q.x, q.y, q.z, q.w);
    }
}


