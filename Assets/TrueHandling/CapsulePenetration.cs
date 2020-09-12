using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Used to compute exact point/normal where a capsule initially penetrates an object
 */

public class CapsulePenetrationData
{
    public Vector3 hitPos;
    public Vector3 hitNorm;
    public Vector3 capsuleDepenetrationTranslation;
    public float capsuleHitFraction;
}

public class CapsulePenetration {

    private static GameObject tester;
    private static CapsuleCollider capsuleCollider;

    // Calculate capsule penetration with automatic depenetration direction from Physics.ComputePenetration
    public static CapsulePenetrationData Test(Vector3 capsulePos, Quaternion capsuleAng, float capsuleLength, float capsuleRadius, Collider other, Vector3 otherPos, Quaternion otherAng)
    {
        PrepareTester(capsuleLength, capsuleRadius);

        CapsulePenetrationData pen = null;
        float dist;
        Vector3 dir;

        if (Physics.ComputePenetration(capsuleCollider, capsulePos, capsuleAng, other, otherPos, otherAng, out dir, out dist))
        {
            pen = MakeCapsulePenetrationData(capsulePos, capsuleAng, capsuleLength, capsuleRadius, other, otherPos, otherAng, dir, dist);
        }

        CleanupTester();

        return pen;
    }

    // Calculate capsule penetration with a known depenetration direction
    public static CapsulePenetrationData TestFixedNormal(Vector3 capsulePos, Quaternion capsuleAng, float capsuleLength, float capsuleRadius, Collider other, Vector3 otherPos, Quaternion otherAng, Vector3 fixedNormal)
    {
        PrepareTester(capsuleLength, capsuleRadius);

        Debug.Assert(fixedNormal.magnitude > 0.99f && fixedNormal.magnitude < 1.01f);

        float dist;
        Vector3 dir;

        float hitDist;
        Vector3 hitDir;

        if (Physics.ComputePenetration(capsuleCollider, capsulePos, capsuleAng, other, otherPos, otherAng, out dir, out dist))
        {
            hitDist = dist;
            hitDir = dir;
        } else
        {
            return null;
        }

        float loDist = 0.0f;
        float hiDist = 0.01f; // default prediction

        while (Physics.ComputePenetration(capsuleCollider, capsulePos + hiDist * fixedNormal, capsuleAng, other, otherPos, otherAng, out dir, out dist))
        {
            hiDist = hiDist * 2.0f;
        }

        // binary search for depenetration distance
        for (int it = 0; it<8; it++)
        {
            float midDist = 0.5f * (loDist + hiDist);
            if (Physics.ComputePenetration(capsuleCollider, capsulePos + midDist * fixedNormal, capsuleAng, other, otherPos, otherAng, out dir, out dist))
            {
                loDist = midDist;
                hitDist = dist;
                hitDir = dir;
            } else
            {
                hiDist = midDist;
            }
        }

        float dnDot = Vector3.Dot(hitDir, fixedNormal);
        if (dnDot > 0f)
        {
            loDist += dnDot * hitDist;
        }

        CapsulePenetrationData pen = MakeCapsulePenetrationData(capsulePos, capsuleAng, capsuleLength, capsuleRadius, other, otherPos, otherAng, fixedNormal, loDist);

        CleanupTester();

        return pen;
    }

    // Calculates exactly how the capsule contacted the object at what point on each
    private static CapsulePenetrationData MakeCapsulePenetrationData(Vector3 capsulePos, Quaternion capsuleAng, float capsuleLength, float capsuleRadius, Collider other, Vector3 otherPos, Quaternion otherAng, Vector3 dir, float dist)
    {
        CapsulePenetrationData pen = new CapsulePenetrationData();

        Debug.Assert(dir.magnitude > 0.99f && dir.magnitude < 1.01f);
        pen.capsuleDepenetrationTranslation = dir * dist;

        pen.hitNorm = dir;
        float dot = Vector3.Dot(dir, capsuleAng * Vector3.forward);
        pen.capsuleHitFraction = dot < 0f ? 1f : 0f;
        if (Mathf.Abs(dot) < 0.0001f)
        {
            // we hit the side of the capsule, not the endpoint
            if (other is BoxCollider || other is SphereCollider || other is CapsuleCollider || (other is MeshCollider && (other as MeshCollider).convex))
            {
                Vector3 projCenter = capsulePos + pen.capsuleDepenetrationTranslation - dir * capsuleRadius;
                Vector3 projAxis = 0.5f * capsuleLength * (capsuleAng * Vector3.forward);
                float lo = -1f;
                float hi = 1f;
                Vector3 loAx = projCenter + lo * projAxis;
                Vector3 hiAx = projCenter + hi * projAxis;
                Vector3 loClose = Physics.ClosestPoint(loAx, other, otherPos, otherAng);
                Vector3 hiClose = Physics.ClosestPoint(hiAx, other, otherPos, otherAng);
                Vector3 approxHitPos = hiClose;
                // binary search
                for (int it = 0; it < 8; it++)
                {
                    if ((loAx - loClose).sqrMagnitude < (hiAx - hiClose).sqrMagnitude)
                    {
                        hi = lo + (hi - lo) * 0.5f;
                        hiAx = projCenter + hi * projAxis;
                        hiClose = Physics.ClosestPoint(hiAx, other, otherPos, otherAng);
                        approxHitPos = loClose;
                    }
                    else
                    {
                        lo = hi + (lo - hi) * 0.5f;
                        loAx = projCenter + lo * projAxis;
                        loClose = Physics.ClosestPoint(loAx, other, otherPos, otherAng);
                        approxHitPos = hiClose;
                    }
                }
                Vector3 p1 = capsulePos + pen.capsuleDepenetrationTranslation + (-0.5f * capsuleLength * (capsuleAng * Vector3.forward)) - dir * capsuleRadius;
                Vector3 p2 = capsulePos + pen.capsuleDepenetrationTranslation + (0.5f * capsuleLength * (capsuleAng * Vector3.forward)) - dir * capsuleRadius;
                Vector3 p1ToCenter = approxHitPos - p1;
                Vector3 p1ToP2 = p2 - p1;
                pen.capsuleHitFraction = Mathf.Clamp01(Vector3.Dot(p1ToP2, p1ToCenter) / p1ToP2.sqrMagnitude);
            }
            else
            {
                pen.capsuleHitFraction = 0.5f;
            }
        }
        pen.hitPos = capsulePos + pen.capsuleDepenetrationTranslation + ((pen.capsuleHitFraction - 0.5f) * capsuleLength * (capsuleAng * Vector3.forward)) - dir * capsuleRadius;

        return pen;
    }

    private static void PrepareTester(float capsuleLength, float capsuleRadius)
    {
        if (tester == null)
        {
            tester = new GameObject("CapsuleTester");
            capsuleCollider = tester.AddComponent<CapsuleCollider>();
            capsuleCollider.direction = 2;
        }

        capsuleCollider.height = capsuleLength + capsuleRadius * 2.0f;
        capsuleCollider.radius = capsuleRadius;

        tester.SetActive(true);
        capsuleCollider.enabled = true;
    }

    private static void CleanupTester()
    {
        tester.SetActive(true);
        capsuleCollider.enabled = true;
    }
}
