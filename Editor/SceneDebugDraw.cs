using UnityEditor;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public class SceneDebugDraw
    {
        public static void DrawPolygon(Fix64Vec3[] verts, int count, Color color)
        {
            Color c = Handles.color;

            Handles.color = color;

            Vector3 previous = Vector3.zero;

            for (int i = 0; i < count; i++)
            {
                Vector3 current = (Vector3)verts[i];
                if (i != 0)
                {
                    Handles.DrawLine(previous, current);
                }

                previous = current;
            }

            Handles.DrawLine(previous, (Vector3)verts[0]);

            Handles.color = c;
        }

        public static void DrawAABB(Fix64Vec3 l, Fix64Vec3 u, Color color)
        {
            Color c = Handles.color;

            Vector3 lower = (Vector3)l;
            Vector3 upper = (Vector3)u;

            Handles.color = color;
            float XL = lower.x;
            float YL = lower.y;
            float ZL = lower.z;

            float XU = upper.x;
            float YU = upper.y;
            float ZU = upper.z;

            Vector3 p0 = new Vector3(XL, YU, ZL);
            Vector3 p1 = new Vector3(XL, YL, ZL);
            Vector3 p2 = new Vector3(XL, YU, ZU);
            Vector3 p3 = new Vector3(XL, YL, ZU);
            Vector3 p4 = new Vector3(XU, YU, ZL);
            Vector3 p5 = new Vector3(XU, YL, ZL);
            Vector3 p6 = new Vector3(XU, YU, ZU);
            Vector3 p7 = new Vector3(XU, YL, ZU);

            Handles.DrawLine(p0, p1);
            Handles.DrawLine(p0, p2);
            Handles.DrawLine(p0, p4);
            Handles.DrawLine(p1, p3);
            Handles.DrawLine(p1, p5);
            Handles.DrawLine(p2, p3);
            Handles.DrawLine(p2, p6);
            Handles.DrawLine(p3, p7);
            Handles.DrawLine(p4, p5);
            Handles.DrawLine(p4, p6);
            Handles.DrawLine(p5, p7);
            Handles.DrawLine(p6, p7);

            Handles.color = c;
        }
    }
}