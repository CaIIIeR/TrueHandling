using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Renders HandState as solid capsules (matching physics)
 */

public class CapsuleHandRenderer : MonoBehaviour {

    public GameObject sourceGameObject;
    public HandStateSource source; // it's an interface so not serializable

    public Mesh cylinder;
    public Mesh sphere;
    public Material material;
    public Material lostMaterial;

    GameObject[,] reprs;

    TrueHandlingPhysics system;

    void Start () {
        HandStateSource[] sources = sourceGameObject.GetComponents<HandStateSource>();
        Debug.Assert(sources.Length == 1);
        source = sources[0];

        reprs = new GameObject[2, HandState.NUM_CAPSULEPARTS * 3];

        system = FindObjectOfType<TrueHandlingPhysics>();
	}
	
	void Update () {
        int gc;
        for (int h=0;h<2;h++)
        {
            HandState state = source.GetHandState(h == 1);
            if (state.active)
            {
                if (reprs[h, 0] == null)
                {
                    for (gc=0; gc<HandState.NUM_CAPSULEPARTS; gc++)
                    {
                        for (int z = 0; z < 2; z++) //do z<3 for back ball
                        {
                            GameObject bon = new GameObject("R");
                            reprs[h, gc*3 + z] = bon;
                            bon.transform.parent = transform;

                            MeshFilter f = bon.AddComponent<MeshFilter>();
                            f.mesh = (z == 0 ? cylinder : sphere);
                            MeshRenderer r = bon.AddComponent<MeshRenderer>();
                            r.material = material;
                        }
                    }
                }

                gc = 0;
                for (int b = 0; b<HandState.NUM_BONES; b++)
                {
                    Vector3 globalPos = state.GetBonePosition(b);
                    Quaternion globalAng = state.GetBoneAngle(b);

                    for (int c = 0; c < state.boneCapsules[b].Length; c++)
                    {
                        CapsulePart dat = state.boneCapsules[b][c];

                        Vector3 capsuleGlobalPos = globalPos + globalAng * dat.boneOffsetPos;
                        Quaternion capsuleGlobalAng = globalAng * dat.boneOffsetAng;

                        for (int z = 0; z < 2; z++)
                        {
                            reprs[h,gc*3+z].transform.rotation = capsuleGlobalAng;

                            MeshRenderer r = reprs[h,gc*3+z].GetComponent<MeshRenderer>();
                            r.material = state.notLost ? (system.boneMaterialOverride[h, b] == null ? material : system.boneMaterialOverride[h, b]) : lostMaterial;

                            if (z == 0)
                            {
                                reprs[h, gc * 3 + z].transform.position = capsuleGlobalPos;
                                reprs[h, gc * 3 + z].transform.Rotate(Vector3.right, 90f); //cylinder mesh's axis is along Y instead of Z like it should be
                                reprs[h, gc * 3 + z].transform.localScale = new Vector3(dat.radius * 2.0f, dat.length * 0.5f, dat.radius * 2.0f);
                            }
                            else
                            {
                                reprs[h, gc * 3 + z].transform.position = capsuleGlobalPos + capsuleGlobalAng * Vector3.forward * (dat.length * 0.5f * (z == 1 ? 1.0f : -1.0f));
                                reprs[h, gc * 3 + z].transform.localScale = new Vector3(dat.radius * 2.0f, dat.radius * 2.0f, dat.radius * 2.0f);
                            }
                        }
                        gc++;
                    }
                }
            } else
            {
                if (reprs[h,0] != null)
                {
                    for (int r = 0; r < HandState.NUM_CAPSULEPARTS*3; r++)
                    {
                        Destroy(reprs[h,r]);
                        reprs[h, r] = null;
                    }
                }
            }
        }
	}
}
