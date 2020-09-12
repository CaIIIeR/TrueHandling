using UnityEngine;
using System.Collections.Generic;

/*
 *Draws debug lines that shows up in the inspector as well as in VR. Got it online somewhere.
 */

public class Gizmoz
{
    public static void DrawLine(Vector3 a, Vector3 b, Color color)
    {
        if (!GizmoManager.Show) return;

        GizmoManager.lines.Add(new GizmoManager.GizmoLine(a, b, color));

        try
        {
            UnityEngine.Gizmos.color = color;
            UnityEngine.Gizmos.DrawLine(a, b);
        }
        catch (System.ArgumentException unityException)
        {
            //ignores the annoying
            //UnityException: Gizmo drawing functions can only be used in OnDrawGizmos and OnDrawGizmosSelected.
            //error
        }
    }

    public static void DrawBox(Vector3 position, Vector3 size, Color color)
    {
        if (!GizmoManager.Show) return;

        Vector3 point1 = new Vector3(position.x - size.x / 2f, position.y - size.y / 2f, position.z - size.z / 2f);
        Vector3 point2 = new Vector3(position.x + size.x / 2f, position.y - size.y / 2f, position.z - size.z / 2f);
        Vector3 point3 = new Vector3(position.x + size.x / 2f, position.y + size.y / 2f, position.z - size.z / 2f);
        Vector3 point4 = new Vector3(position.x - size.x / 2f, position.y + size.y / 2f, position.z - size.z / 2f);

        Vector3 point5 = new Vector3(position.x - size.x / 2f, position.y - size.y / 2f, position.z + size.z / 2f);
        Vector3 point6 = new Vector3(position.x + size.x / 2f, position.y - size.y / 2f, position.z + size.z / 2f);
        Vector3 point7 = new Vector3(position.x + size.x / 2f, position.y + size.y / 2f, position.z + size.z / 2f);
        Vector3 point8 = new Vector3(position.x - size.x / 2f, position.y + size.y / 2f, position.z + size.z / 2f);

        DrawLine(point1, point2, color);
        DrawLine(point2, point3, color);
        DrawLine(point3, point4, color);
        DrawLine(point4, point1, color);

        DrawLine(point5, point6, color);
        DrawLine(point6, point7, color);
        DrawLine(point7, point8, color);
        DrawLine(point8, point5, color);

        DrawLine(point1, point5, color);
        DrawLine(point2, point6, color);
        DrawLine(point3, point7, color);
        DrawLine(point4, point8, color);
    }

    public static void DrawSquare(Vector3 position, Vector3 size, Color color)
    {
        if (!GizmoManager.Show) return;

        Vector3 point1 = new Vector3(position.x - size.x / 2f, position.y - size.y / 2f, position.z);
        Vector3 point2 = new Vector3(position.x + size.x / 2f, position.y - size.y / 2f, position.z);
        Vector3 point3 = new Vector3(position.x + size.x / 2f, position.y + size.y / 2f, position.z);
        Vector3 point4 = new Vector3(position.x - size.x / 2f, position.y + size.y / 2f, position.z);

        DrawLine(point1, point2, color);
        DrawLine(point2, point3, color);
        DrawLine(point3, point4, color);
        DrawLine(point4, point1, color);
    }

    public static void DrawCircle(Vector3 position, float radius, Color color)
    {
        if (!GizmoManager.Show) return;

        DrawPolygon(position, radius, 18, color);
    }

    public static void DrawPolygon(Vector3 position, float radius, int points, Color color)
    {
        if (!GizmoManager.Show) return;

        float angle = 360f / points;

        for (int i = 0; i < points; ++i)
        {
            float sx = Mathf.Cos(Mathf.Deg2Rad * angle * i) * radius / 2;
            float sy = Mathf.Sin(Mathf.Deg2Rad * angle * i) * radius / 2;

            float nx = Mathf.Cos(Mathf.Deg2Rad * angle * (i + 1)) * radius / 2;
            float ny = Mathf.Sin(Mathf.Deg2Rad * angle * (i + 1)) * radius / 2;

            Vector3 a = new Vector3(sx, sy, position.z);
            Vector3 b = new Vector3(nx, ny, position.z);

            DrawLine(position + a, position + b, color);
        }
    }
}

//Must be attached to a camera.
public class GizmoManager : MonoBehaviour
{
    public struct GizmoLine
    {
        public Vector3 a;
        public Vector3 b;
        public Color color;

        public GizmoLine(Vector3 a, Vector3 b, Color color)
        {
            this.a = a;
            this.b = b;
            this.color = color;
        }
    }

    public bool showGizmos = true;
    public Material material;
    internal static List<GizmoLine> lines = new List<GizmoLine>();
    internal static List<GizmoLine> drawlines = new List<GizmoLine>();
    bool fixd = true;

    public static bool Show { get; private set; }

    private void Update()
    {
        Show = showGizmos;
    }

    private void FixedUpdate()
    {
        fixd = true;
    }

    void OnPostRender()
    {

        if (fixd)
        {
            drawlines = lines;
            lines = new List<GizmoLine>();
            fixd = false;
        }

        material.SetPass(0);
        GL.Begin(GL.LINES);

        for (int i = 0; i < drawlines.Count; i++)
        {
            GL.Color(drawlines[i].color);
            GL.Vertex(drawlines[i].a);
            GL.Vertex(drawlines[i].b);
        }

        GL.End();
    }
}