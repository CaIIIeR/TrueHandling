using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class LeapCalibrationHandController : MonoBehaviour {

    public Transform puller_prefab;
    public bool right;
    string hand;

	void Start () {
        hand = right ? "Right" : "Left";
    }
	
	void Update () {
        List<XRNodeState> nodes = new List<XRNodeState>();
        InputTracking.GetNodeStates(nodes);

        bool notfound = true;
        foreach (XRNodeState node in nodes)
        {
            string name = InputTracking.GetNodeName(node.uniqueID);
            if (name.Contains("OpenVR Controller") && name.Contains(hand)) {
                notfound = false;
                Vector3 pos;
                Quaternion ang;
                bool ok = node.TryGetPosition(out pos);
                ok &= node.TryGetRotation(out ang);
                if (ok)
                {
                    transform.localPosition = pos;
                    transform.localRotation = ang;
                }   
            }
        }

        if (notfound)
        {
            print("Couldn't find controller. You must be in OpenVR (not Oculus) mode.");
        }
    }
}
