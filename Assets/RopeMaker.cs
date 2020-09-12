using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* 
 * Creates a physically simulated rope with a skinned mesh
 * Made for TrueHandling but should work separately
 */

public class RopeMaker : MonoBehaviour {

    public GameObject other;

    // local positions to which rope is attached
    public Vector3 offset1;
    public Vector3 offset2;

    public bool noCollide = false;

    // these are only for drawing the rope; they don't do anything physically
    public bool useCustomAttachNormals = false;
    public Vector3 attachNormal1;
    public Vector3 attachNormal2;

    public float length = 0.5f;

    public float radius = 0.005f;

    // extra extension at attach points
    //public float extensionIntoSurface = 0.02f;

    // reduces velocity
    public float drag = 0.0f;
    // reduces jitter in the rope
    public float angularDrag = 100.0f;

    // extra length, private because it works better if you dont use it
    private float play = 0f;

    // seems to only work with play
    private float bounce = 0f;

    // enter 5 to get 32 joints
    public int jointsPowerTwo = 6;

    public float ropeMass = 2f;

    // circular slices per segment, 3 is best
    public int meshQuality = 3;

    // rotation steps
    public int meshRoundness = 8;

    public Material mat;

    private int joints;
    private int parts;
    private GameObject[] segments;
    private SkinnedMeshRenderer ropeRenderer;

    private Vector3[] curvpoints;
    private Vector3[] points;

    private Transform[] bones;

    private bool iter;

    void Start () {
        if (!useCustomAttachNormals)
        {
            attachNormal1 = offset1;
            attachNormal2 = offset2;
        }
        if (attachNormal1.Equals(Vector3.zero)) { attachNormal1 = Vector3.up; }
        if (attachNormal2.Equals(Vector3.zero)) { attachNormal2 = Vector3.up; }
        attachNormal1 = attachNormal1.normalized;
        attachNormal2 = attachNormal2.normalized;

        GameObject rope = new GameObject("ROPE");
        rope.transform.parent = null;

        joints = 1;
        for (int i=0; i<jointsPowerTwo; i++)
        {
            joints *= 2;
        }
        parts = joints - 1;
        segments = new GameObject[parts];

        float ifparts = 1.0f / ((float)parts);
        float lengthStep = length * ifparts;

        Vector3 pos1 = gameObject.transform.TransformPoint(offset1);
        Vector3 pos2 = other.transform.TransformPoint(offset2);

        for (int i = 0; i<parts; i++)
        {
            segments[i] = new GameObject("ROPE");
            segments[i].transform.parent = rope.transform;

            float lerp = ((float)i) / ((float)(parts));

            segments[i].transform.position = pos1 * (1.0f - lerp) + pos2 * lerp;
            segments[i].transform.LookAt(pos2);

            CapsuleCollider sphere = segments[i].AddComponent<CapsuleCollider>();
            sphere.radius = radius;
            sphere.direction = 2;
            sphere.height = 2 * radius + lengthStep;
            sphere.center = new Vector3(0, 0, lengthStep * 0.5f);
            Rigidbody spherebody = segments[i].AddComponent<Rigidbody>();
            spherebody.drag = drag;
            spherebody.angularDrag = angularDrag;

            spherebody.mass = ropeMass * ifparts;

            if (noCollide)
            {
                spherebody.detectCollisions = false;
            }
        }

        for (int stride = 1; stride <= joints; stride*=2)
        {
            for (int i = 0; i < joints; i+=stride)
            {
                int i2 = i + stride;
                GameObject first = (i == 0) ? gameObject : segments[i - 1];
                GameObject second = (i2 < joints) ? segments[i2 - 1] : other;

                ConfigurableJoint joint = first.AddComponent<ConfigurableJoint>();
                joint.autoConfigureConnectedAnchor = false;

                joint.connectedBody = second.GetComponent<Rigidbody>();

                joint.anchor = (i == 0) ? offset1 : new Vector3(0, 0, lengthStep);
                joint.connectedAnchor = (i2 == joints) ? offset2 : Vector3.zero;

                // attached to world
                if (joint.connectedBody == null)
                {
                    joint.connectedAnchor = second.transform.TransformPoint(offset2);
                }

                float thisPlay = play;
                if (stride > 1)
                {
                    thisPlay += length;
                    joint.enableCollision = true;
                }

                if (thisPlay > 0f)
                {
                    SoftJointLimit slimit = joint.linearLimit;
                    slimit.limit = stride * thisPlay / ((float)(joints));
                    slimit.bounciness = bounce;
                    joint.linearLimit = slimit;

                    joint.xMotion = ConfigurableJointMotion.Limited;
                    joint.yMotion = ConfigurableJointMotion.Limited;
                    joint.zMotion = ConfigurableJointMotion.Limited;
                } else
                {
                    joint.xMotion = ConfigurableJointMotion.Locked;
                    joint.yMotion = ConfigurableJointMotion.Locked;
                    joint.zMotion = ConfigurableJointMotion.Locked;
                }
            }
        }

        Mesh mesh = new Mesh();

        Vector3[] vertices = new Vector3[(meshQuality * parts + 1) * meshRoundness];
        Vector2[] uvs = new Vector2[vertices.Length];
        BoneWeight[] weights = new BoneWeight[vertices.Length];
        int[] triangles = new int[meshQuality * parts * meshRoundness * 6];

        float circum = 2f * Mathf.PI * radius;
        for (int j = 0; j<= parts; j++)
        {
            for (int q = 0; q < (j == parts ? 1 : meshQuality); q++)
            {
                for (int r = 0; r < meshRoundness; r++)
                {
                    float ang = 2f * Mathf.PI * (float)r / (float)meshRoundness;

                    int vertidx = j * meshQuality * meshRoundness + q * meshRoundness + r;

                    vertices[vertidx].x = Mathf.Sin(ang) * radius;
                    vertices[vertidx].y = Mathf.Cos(ang) * radius;
                    vertices[vertidx].z = length * (float)(j * meshQuality + q) / (float)(parts * meshQuality);

                    uvs[vertidx].x = vertices[vertidx].z / circum;
                    uvs[vertidx].y = (float)r / (float)meshRoundness;

                    float boneIndexer = (float)j + ((float)(q) / (float)meshQuality) + 0.5f;

                    if (boneIndexer < 1)
                    {
                        //vertices[vertidx].z += boneIndexer * 2f * extensionIntoSurface;
                        boneIndexer = 1f - (1f - boneIndexer) * 2f;
                    }
                    if (boneIndexer > parts)
                    {
                        //vertices[vertidx].z += (boneIndexer - (float)(parts-1)) * 2f * extensionIntoSurface;
                        boneIndexer = (boneIndexer - parts) * 2f + parts;
                    }

                    weights[vertidx].boneIndex0 = Mathf.FloorToInt(boneIndexer);

                    weights[vertidx].boneIndex1 = weights[vertidx].boneIndex0 + 1;
                    weights[vertidx].weight1 = boneIndexer % 1f;
                    weights[vertidx].weight0 = 1f - weights[vertidx].weight1;

                    if (weights[vertidx].boneIndex1 >= parts+1)
                    {
                        weights[vertidx].boneIndex1 = 0;
                        weights[vertidx].weight0 = 1f;
                        weights[vertidx].weight1 = 0f;
                    }

                    if (j < parts)
                    {
                        int circnext = vertidx + 1 - ((r == meshRoundness - 1) ? meshRoundness : 0);
                        int triidx = vertidx * 6;
                        triangles[triidx] = vertidx;
                        triangles[triidx+2] = circnext;
                        triangles[triidx+1] = vertidx + meshRoundness;
                        triangles[triidx + 3] = circnext;
                        triangles[triidx + 4] = vertidx + meshRoundness;
                        triangles[triidx + 5] = circnext + meshRoundness;
                    }
                }
            }
        }

        Matrix4x4[] bindPoses = new Matrix4x4[parts + 2];

        for (int i = 0; i < parts + 2; i++)
        {
            float pi = Mathf.Max(i - 1, 0f);
            bindPoses[i] = Matrix4x4.TRS(new Vector3(0, 0, -pi * lengthStep), Quaternion.identity, Vector3.one);
        }

        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.boneWeights = weights;
        mesh.bindposes = bindPoses;
        mesh.RecalculateNormals();

        bones = new Transform[parts + 2];

        for (int i = 0; i < parts+2; i++)
        {
            GameObject boneobj = new GameObject("ROPERENDERBONE");
            boneobj.transform.parent = rope.transform;
            bones[i] = boneobj.transform;
        }
        bones[0].transform.parent = gameObject.transform;
        bones[0].localPosition = offset1;
        bones[0].localRotation = Quaternion.LookRotation(attachNormal1, Vector3.up);

        GameObject renderobj = new GameObject("ROPERENDER");
        renderobj.transform.parent = rope.transform;

        ropeRenderer = renderobj.AddComponent<SkinnedMeshRenderer>();
        ropeRenderer.quality = UnityEngine.SkinQuality.Bone2;
        ropeRenderer.material = mat;
        ropeRenderer.sharedMesh = mesh;
        ropeRenderer.bones = bones;
        ropeRenderer.updateWhenOffscreen = true;

        /* catmullrom linerenderer
        ropeRenderer = renderobj.AddComponent<LineRenderer>();
        ropeRenderer.useWorldSpace = true;
        ropeRenderer.startWidth = radius * 2f;
        ropeRenderer.endWidth = radius * 2f;
        Material[] mats = new Material[1];
        mats[0] = mat;
        ropeRenderer.materials = mats;

        curvpoints = new Vector3[parts + 1];
        points = new Vector3[parts * lineQuality + 1];

        ropeRenderer.positionCount = points.Length;
        */
        LateUpdate();
    }

    void LateUpdate () {
        // lines up the bones without twisting
        for (int i = 1; i < bones.Length; i++)
        {
            Vector3 lastAxis = bones[i - 1].forward;
            Vector3 nextAxis = (i < bones.Length - 1) ? segments[i-1].transform.forward : -other.transform.TransformVector(attachNormal2);
            Vector3 cp = Vector3.Cross(lastAxis, nextAxis);
            bones[i].transform.rotation = bones[i - 1].rotation;
            if (cp.sqrMagnitude > 0)
            {
                bones[i].transform.RotateAround(Vector3.zero, cp.normalized, Vector3.Angle(lastAxis, nextAxis));
            }
            bones[i].transform.position = (i < bones.Length - 1) ? segments[i-1].transform.position : other.transform.TransformPoint(offset2);
        }

        /* catmullrom linerenderer
        curvpoints[0] = gameObject.transform.TransformPoint(offset1);
        curvpoints[parts] = other.transform.TransformPoint(offset2);
        for (int i = 1; i < parts; i++)
        {
            curvpoints[i] = segments[i].transform.position;
        }
        int idx = 0;
        for (int i = 0; i < parts; i++)
        {
            for (int j=0;j<lineQuality;j++)
            {
                Vector3 p0 = curvpoints[i > 0 ? i - 1 : i];
                Vector3 p1 = curvpoints[i];
                Vector3 p2 = curvpoints[i + 1];
                Vector3 p3 = curvpoints[i == parts - 1 ? i + 1 : i + 2];

                float t = ((float)j) / ((float)lineQuality);

                points[idx] = 0.5f * ((2f * p1) + (p2 - p0) * t + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t + (3f * p1 + p3 - (3f * p2 + p0)) * t * t * t);

                idx++;
            }
        }
        points[points.Length - 1] = curvpoints[curvpoints.Length - 1];

        ropeRenderer.SetPositions(points); */
    }
}
