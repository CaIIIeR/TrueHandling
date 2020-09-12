using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.XR;

/* 
 * Calculates Leap Motion sensor offset from headset based on controller positions
 * Only set up/tested with Oculus CV1
 * Made for TrueHandling but is adaptable to work with stock Leap Motion interaction engine
 */

public class LeapCalibrationMain : MonoBehaviour {

    GameObject ltruth;
    GameObject rtruth;
    GameObject lmeasured;
    GameObject rmeasured;

    GameObject leapOrigin;

    bool lastTriggerDown = false;

    List<Vector3> truths;
    List<Vector3> measurements;

    //Leap.Controller controller;
    Leap.Unity.LeapServiceProvider _provider;

    // Use this for initialization
    void Start () {
        ltruth = GameObject.Find("LeapCalibrationTruthL");
        rtruth = GameObject.Find("LeapCalibrationTruthR");
        lmeasured = GameObject.Find("LeapCalibrationMeasuredL");
        rmeasured = GameObject.Find("LeapCalibrationMeasuredR");
        leapOrigin = GameObject.Find("LeapCalibratedOrigin");

        leapOrigin.transform.SetParent(Camera.main.transform);
        leapOrigin.transform.localPosition = Vector3.zero;
        leapOrigin.transform.localRotation = Quaternion.identity;

        try
        {
            StreamReader reader = new StreamReader("stored_calibration.txt");
            String[] parts = reader.ReadToEnd().Split(' ');
            leapOrigin.transform.localPosition = new Vector3(float.Parse(parts[0]), float.Parse(parts[1]), float.Parse(parts[2]));
            leapOrigin.transform.localRotation = new Quaternion(float.Parse(parts[3]), float.Parse(parts[4]), float.Parse(parts[5]), float.Parse(parts[6]));
        } catch (FileNotFoundException e) { }


        foreach (var provider in FindObjectsOfType<Leap.Unity.LeapXRServiceProvider>())
        {
            provider.deviceOrigin = leapOrigin.transform;
            provider.deviceOffsetMode = Leap.Unity.LeapXRServiceProvider.DeviceOffsetMode.Transform;

            _provider = provider;
        }
    }
	
	void Update () {
        Leap.Frame frame = _provider.CurrentFrame;

        if (frame != null)
        {
            foreach (Leap.Hand hand in frame.Hands)
            {
                GameObject measured = hand.IsLeft ? lmeasured : rmeasured;

                Leap.Bone tip = hand.Fingers[0].Bone(Leap.Bone.BoneType.TYPE_DISTAL);

                // why is this necessary
                Leap.Vector v = tip.NextJoint;
                Vector3 uv = new Vector3(v.x, v.y, v.z);

                Leap.LeapQuaternion q = tip.Rotation;
                Quaternion uq = new Quaternion(q.x, q.y, q.z, q.w);

                measured.transform.position = uv;
                measured.transform.rotation = uq;
            }
        }

        bool lTriggerDown = Input.GetAxis("LeftVRTriggerAxis") > 0.5;
        bool rTriggerDown = Input.GetAxis("RightVRTriggerAxis") > 0.5;
        bool triggerDown = lTriggerDown || rTriggerDown;

        ltruth.SetActive(lTriggerDown);
        rtruth.SetActive(rTriggerDown);
        lmeasured.SetActive(lTriggerDown);
        rmeasured.SetActive(rTriggerDown);

        if (triggerDown)
        {
            if (!lastTriggerDown)
            {
                leapOrigin.transform.localPosition = Vector3.zero;
                leapOrigin.transform.localRotation = Quaternion.identity;

                truths = new List<Vector3>();
                measurements = new List<Vector3>();                
            } else
            {
                Transform cam = Camera.main.transform;

                if (lmeasured.activeInHierarchy)
                {
                    truths.Add(cam.InverseTransformPoint(ltruth.transform.position));
                    measurements.Add(cam.InverseTransformPoint(lmeasured.transform.position));
                }
                if (rmeasured.activeInHierarchy)
                {
                    truths.Add(cam.InverseTransformPoint(rtruth.transform.position));
                    measurements.Add(cam.InverseTransformPoint(rmeasured.transform.position));
                }
            }
        } else
        {
            if (lastTriggerDown)
            {

                Vector3 truthCentroid = new Vector3(0, 0, 0);
                Vector3 measurementCentroid = new Vector3(0, 0, 0);
                
                int l = truths.Count;

                foreach (Vector3 p in measurements)
                {
                    measurementCentroid += p;
                }
                measurementCentroid /= truths.Count;

                foreach (Vector3 p in truths)
                {
                    truthCentroid += p;
                }
                truthCentroid /= truths.Count;
                

                double[,] A = new double[3,l];
                double[,] B = new double[3,l];

                int i = 0;
                foreach (Vector3 p in measurements)
                {
                    Vector3 po = p - measurementCentroid;
                    A[0, i] = po.x;
                    A[1, i] = po.y;
                    A[2, i] = po.z;
                    i++;
                }
                i = 0;
                foreach (Vector3 p in truths)
                {
                    Vector3 po = p - truthCentroid;
                    B[0, i] = po.x;
                    B[1, i] = po.y;
                    B[2, i] = po.z;
                    i++;
                }
                double[,] BAT = new double[3,3];
                alglib.rmatrixgemm(3, 3, l, 1, B, 0, 0, 0, A, 0, 0, 1, 0, ref BAT, 0, 0);

                double[] W = new double[3];
                double[,] U = new double[3, 3];
                double[,] VT = new double[3, 3];

                alglib.rmatrixsvd(BAT, 3, 3, 2, 2, 2, out W, out U, out VT);

                double[,] UVT = new double[3, 3];
                alglib.rmatrixgemm(3, 3, 3, 1, U, 0, 0, 0, VT, 0, 0, 0, 0, ref UVT, 0, 0);

                double w = Math.Sqrt(1 + UVT[0, 0] + UVT[1, 1] + UVT[2, 2]) / 2.0;
                double iw4 = 1.0/(w * 4);
                Quaternion q = new Quaternion((float) ((UVT[2, 1] - UVT[1, 2]) * iw4),
                    (float)((UVT[0, 2] - UVT[2, 0]) * iw4),
                    (float)((UVT[1, 0] - UVT[0, 1]) * iw4), (float)w);
                Quaternion iq = Quaternion.Inverse(q);

                Vector3 mtt = truthCentroid - measurementCentroid;

                leapOrigin.transform.localPosition = truthCentroid - q * measurementCentroid;
                leapOrigin.transform.localRotation = q;

                StreamWriter writer = new StreamWriter("stored_calibration.txt");
                writer.Write(String.Format("{0} {1} {2} {3} {4} {5} {6}",
                    leapOrigin.transform.localPosition.x,
                    leapOrigin.transform.localPosition.y,
                    leapOrigin.transform.localPosition.z,
                    leapOrigin.transform.localRotation.x,
                    leapOrigin.transform.localRotation.y,
                    leapOrigin.transform.localRotation.z,
                    leapOrigin.transform.localRotation.w
                ));
                writer.Close();

                truths = null;
                measurements = null;
            }
        }

        lastTriggerDown = triggerDown;

    }
}
