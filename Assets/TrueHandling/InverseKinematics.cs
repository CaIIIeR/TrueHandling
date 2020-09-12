using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * FABRIK implementation for TrueHandling
 * IS NOT SET UP OR WORKING RIGHT NOW
 */

public class FABRIK
{
    public static void SimpleBackward(HandState[] states)
    {
        foreach (HandState state in states)
        {
            if (!state.active)
            {
                continue;
            }

            SimpleBackwardStep(state, HandState.POINT_ELBOW);
        }
    }

    private static void SimpleBackwardStep(HandState state, int p) {
        foreach (int c in HandState.POINT_CHILDREN[p])
        {
            Vector3 p2c = state.points[c] - state.points[p];
            p2c.Normalize();
            state.points[c] = state.points[p] * state.pointDistanceFromParent[c];
            SimpleBackwardStep(state, c);
        }
    }


    public static void Solve(HandState[] states, IKPointTarget[] pointTargets, IKGapTarget[] gapTargets)
    {
        if (!states[0].active && !states[1].active)
        {
            return;
        }

        FABRIK data = new FABRIK(states, pointTargets, gapTargets);
        for (int i = 0; i<5; i++) {
            data.SolveStep();
        }
        data.Apply(states);
    }

    private class IKRoot
    {
        public int p;
        public Vector3 pos;

        public IKRoot(int p_, Vector3 pos_)
        {
            p = p_;
            pos = pos_;
        }
    }

    private class IKPoint
    {
        public int parent;
        public float parentDistance;

        public Dictionary<int, float> childDistance;
        public Dictionary<int, float> siblingDistance;

        public Vector3 pos;

        public IKAccumulator childAccumulator;
        public IKAccumulator siblingAccumulator;

        public int outputH;
        public int outputP;

        public bool propogatedToParent;
        public HashSet<int> propogatedToSiblings;

        public bool didBackward;

        public IKPoint()
        {
            parent = -1;
            outputH = -1;
            outputP = -1;

            childDistance = new Dictionary<int, float>();
            siblingDistance = new Dictionary<int, float>();

            childAccumulator = new IKAccumulator();
            siblingAccumulator = new IKAccumulator();
        }

        public Vector3 GetChildAccumulation()
        {
            if (childAccumulator.Count()==0)
            {
                return pos;
            } else
            {
                return childAccumulator.Get();
            }
        }

        public Vector3 GetSiblingAccumulation()
        {
            if (siblingAccumulator.Count() == 0)
            {
                return GetChildAccumulation();
            }
            else
            {
                return siblingAccumulator.Get();
            }
        }

        public void BeginForward()
        {
            childAccumulator.Reset();
            siblingAccumulator.Reset();
            propogatedToParent = false;
            propogatedToSiblings = new HashSet<int>();
            didBackward = (parent == -1);
        }
    }

    private class IKAccumulator
    {
        private Vector3 pos;
        private int count;

        public void Reset()
        {
            pos = Vector3.zero;
            count = 0;
        }

        public void Add(Vector3 p)
        {
            pos += p;
            count++;
        }

        public int Count()
        {
            return count;
        }

        public Vector3 Get()
        {
            return pos / (float)count;
        }
    }

    int[,] pidTranslation;
    Vector3[,] fingerPlanes;
    IKPoint[] points;
    IKRoot[] forwardRoots; //world space constraints

    private FABRIK(HandState[] states, IKPointTarget[] pointTargets, IKGapTarget[] gapTargets)
    {
        
        int activeHands = (states[0].active ? 1 : 0) + (states[1].active ? 1 : 0);
        Debug.Assert(activeHands > 0);

        int totalPoints = activeHands * HandState.NUM_POINTS + pointTargets.Length + gapTargets.Length * 2;

        points = new IKPoint[totalPoints];

        pidTranslation = new int[2, HandState.NUM_POINTS];
        fingerPlanes = new Vector3[2, 5];
        for (int h = 0; h<2; h++)
        {
            for (int ip = 0; ip < HandState.NUM_POINTS; ip++)
            {
                pidTranslation[h, ip] = -1;
            }
            for (int f = 0; f<5; f++)
            {
                fingerPlanes[h, f] = Vector3.zero;
            }
        }

        int p = 0;
        for (int h = 0; h < 2; h++)
        {
            HandState state = states[h];
            if (state.active)
            {
                for (int ip = 0; ip < HandState.NUM_POINTS; ip++)
                {
                    IKPoint pt = points[p] = new IKPoint();
                    pidTranslation[h, ip] = p;
                    pt.pos = state.points[ip];
                    pt.outputH = h;
                    pt.outputP = ip;
                    p++;
                }

                for (int ip = 0; ip < HandState.NUM_POINTS; ip++)
                {
                    if (HandState.POINT_PARENT[ip] != -1)
                    {
                        points[pidTranslation[h, ip]].parent = pidTranslation[h, HandState.POINT_PARENT[ip]];
                    }
                }
            }
        }

        forwardRoots = new IKRoot[pointTargets.Length];

        for (int i = 0; i<pointTargets.Length; i++)
        {
            IKPointTarget targ = pointTargets[i];

            Debug.Assert(states[targ.h].active);

            forwardRoots[i] = new IKRoot(p, targ.worldTargetPos);

            //anchor for the root
            IKPoint pt = points[p] = new IKPoint();
            pt.parent = pidTranslation[targ.h, HandState.POINT_PARENT[targ.b]];
            
            //put it in the local pos for initial distance constraints, the roots will move it later
            pt.pos = states[targ.h].LocalToWorld(targ.b, targ.localPos);

            p++;
        }


        for (int i = 0; i < gapTargets.Length; i++)
        {
            IKGapTarget targ = gapTargets[i];

            Debug.Assert(states[targ.h1].active);
            Debug.Assert(states[targ.h2].active);

            //anchor for the root
            IKPoint pt = points[p] = new IKPoint();
            pt.parent = pidTranslation[targ.h1, HandState.POINT_PARENT[targ.b1]];
            pt.pos = states[targ.h1].LocalToWorld(targ.b1, targ.localPos1);

            pt.siblingDistance[p + 1] = targ.dist;

            p++;

            pt = points[p] = new IKPoint();
            pt.parent = pidTranslation[targ.h2, HandState.POINT_PARENT[targ.b2]];
            pt.pos = states[targ.h2].LocalToWorld(targ.b2, targ.localPos2);

            pt.siblingDistance[p - 1] = targ.dist;

            p++;
        }

        Debug.Assert(p == totalPoints);

        //setup parent/child lengths
        for (p=0; p<totalPoints; p++)
        {
            IKPoint pt = points[p];
            if (pt.parent != -1)
            {
                IKPoint parent = points[pt.parent];
                pt.parentDistance = Vector3.Distance(pt.pos, parent.pos);
                parent.childDistance.Add(p, pt.parentDistance);
            }
        }

        //setup sibling lengths
        for (p = 0; p < totalPoints; p++)
        {
            IKPoint pt = points[p];
            if (pt.parent != -1)
            {
                IKPoint parent = points[pt.parent];
                foreach(var kv in parent.childDistance)
                {
                    int sibling = kv.Key;
                    if (sibling != p)
                    {
                        pt.siblingDistance[sibling] = Vector3.Distance(pt.pos, points[sibling].pos);
                    }
                }
            }
        }
    }

    private void SolveStep()
    {
        for (int p = 0; p<points.Length; p++)
        {
            points[p].BeginForward();
        }

        for (int fr = 0; fr<forwardRoots.Length; fr++)
        {
            IKRoot fwd = forwardRoots[fr];
            points[fwd.p].pos = fwd.pos;
        }

        //fabrik forward, repeat till we have found nothing to do

        bool done = false;
        while(!done)
        {
            done = true;

            for (int p = 0; p<points.Length; p++)
            {
                IKPoint pt = points[p];
                if (pt.propogatedToSiblings.Count < pt.siblingDistance.Count)
                {
                    done = false;

                    //all children have reported:
                    if (pt.childAccumulator.Count() == pt.childDistance.Count)
                    {
                        foreach (var kv in pt.siblingDistance)
                        {
                            int s = kv.Key;

                            if (!pt.propogatedToSiblings.Contains(s))
                            {
                                IKPoint sibling = points[s];

                                //all its children have reported too
                                if (sibling.childAccumulator.Count() == sibling.childDistance.Count)
                                {
                                    Vector3 us = pt.GetChildAccumulation();
                                    Vector3 them = sibling.GetChildAccumulation();

                                    Vector3 us2them = them - us;
                                    Vector3 midpoint = 0.5f * (us + them);

                                    //add our thing to their accumulator
                                    sibling.siblingAccumulator.Add(midpoint + (0.5f * kv.Value * us2them.normalized));

                                    pt.propogatedToSiblings.Add(s);
                                }
                                
                            }
                            
                        }
                    }
                }
                if (!pt.propogatedToParent)
                {
                    done = false;

                    //all children and siblings have reported
                    if (pt.childAccumulator.Count() == pt.childDistance.Count && pt.siblingAccumulator.Count() == pt.siblingDistance.Count)
                    {
                        pt.propogatedToParent = true;

                        if (pt.parent != -1)
                        {
                            IKPoint parent = points[pt.parent];

                            //update pos except if we are the root
                            pt.pos = pt.GetSiblingAccumulation();
                            Vector3 us2them = parent.pos - pt.pos;

                            parent.childAccumulator.Add(pt.pos + (pt.parentDistance * us2them.normalized));
                        }
                    }

                }
            }
        }


        //do finger constraints - move tip into plane defined by joints
        for (int h = 0; h<2; h++)
        {
            int idx = pidTranslation[h, HandState.POINT_KNUCKLE[0]];
            if (idx==-1) { continue; }

            for (int f = 0; f<5; f++)
            {
                Vector3 knuckle = points[pidTranslation[h, HandState.POINT_KNUCKLE[f]]].pos;
                Vector3 v1 = points[pidTranslation[h, HandState.POINT_LOJOINT[f]]].pos - knuckle;
                Vector3 v2 = points[pidTranslation[h, HandState.POINT_TIP[f]]].pos - knuckle;

                Vector3 pn = Vector3.Cross(v1.normalized, v2.normalized);

                if (pn.sqrMagnitude < 0.001)
                {
                    fingerPlanes[h, f] = Vector3.zero;
                    continue;
                }

                fingerPlanes[h, f] = pn.normalized;
            }
        }

        //do backward
        done = false;
        while (!done)
        {
            done = true;

            for (int p = 0; p < points.Length; p++)
            {
                IKPoint pt = points[p];

                if (!pt.didBackward)
                {
                    done = false;

                    IKPoint parent = points[pt.parent];

                    if (parent.didBackward)
                    {
                        Vector3 planeConstraint = Vector3.zero;
                        if (pt.outputH != -1 && pt.outputP != -1 && pt.outputP < 20 && pt.outputP%4>0)
                        {
                            planeConstraint = fingerPlanes[pt.outputH, pt.outputP / 4];
                        }

                        Vector3 them2us = pt.pos - parent.pos;
                        
                        if (planeConstraint.sqrMagnitude > 0.99f)
                        {
                            Debug.Assert(planeConstraint.sqrMagnitude < 1.01f);
                            them2us -= Vector3.Dot(them2us, planeConstraint) * planeConstraint;
                        }

                        pt.pos = parent.pos + (pt.parentDistance * them2us.normalized);
                        pt.didBackward = true;
                    }
                }
            }
        }

    }

    private void Apply(HandState[] states)
    {
        for (int p = 0; p < points.Length; p++)
        {
            IKPoint pt = points[p];

            if (pt.outputH != -1)
            {
                states[pt.outputH].points[pt.outputP] = pt.pos;
            }
        }
    }

}

public class IKPointTarget
{
    public int h;
    public int b;
    public Vector3 localPos;
    public Vector3 worldTargetPos;
}

public class IKGapTarget
{
    public int h1;
    public int h2;
    public int b1;
    public int b2;
    public Vector3 localPos1;
    public Vector3 localPos2;
    public float dist;
}

/*
public class StretchedContactData
{
    //no longer needed because of contactspheres
    public static float IKPADDING = 0.0f; //005f;

    public Vector3 objAttachedGlobalCapsuleAxisPos;
    public Vector3 boneAttachedGlobalCapsuleAxisPos;
    public Vector3 boneLocalCapsuleAxisPos;
    //public Vector3 handLocalPos;
    //public Vector3 handWorldPos;
    public int h;
    public int b;
    public int ci;

    public StretchedContactData(HandContact ccp, CapsulePart part, GameObject obj, Vector3 boneGlobalPos, Quaternion boneGlobalAng, int h_, int b_, int ci_)
    {
        //objLocalPos = cpp.otherContact;
        //handLocalPos = cpp.localContact;
        //handWorldPos = globalPos + globalAng * cpp.localContact;
        h = h_;
        b = b_;
        ci = ci_;

        Transform objTf = obj.transform;

        objAttachedGlobalCapsuleAxisPos = objTf.TransformPoint(ccp.otherPos) + (IKPADDING + part.radius) * objTf.TransformDirection(ccp.otherNormal);
        boneLocalCapsuleAxisPos = part.AxisFractionToBonePosition(ccp.axisFraction);
        boneAttachedGlobalCapsuleAxisPos = boneGlobalPos + boneGlobalAng * boneLocalCapsuleAxisPos;
    }

    public bool IsConnected(StretchedContactData other, float allowance)
    {
        return Vector3.Distance(boneAttachedGlobalCapsuleAxisPos, other.boneAttachedGlobalCapsuleAxisPos) <= Vector3.Distance(objAttachedGlobalCapsuleAxisPos, other.objAttachedGlobalCapsuleAxisPos) + allowance;
    }

    public IKGapTarget GetGap(StretchedContactData other)
    {
        IKGapTarget gap = new IKGapTarget();
        gap.h1 = h;
        gap.h2 = other.h;
        gap.b1 = b;
        gap.b2 = other.b;
        gap.localPos1 = boneLocalCapsuleAxisPos;
        gap.localPos2 = other.boneLocalCapsuleAxisPos;
        gap.dist = Vector3.Distance(objAttachedGlobalCapsuleAxisPos, other.objAttachedGlobalCapsuleAxisPos); // + 0.005f;
        return gap;
    }
}*/

