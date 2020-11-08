using System;
using System.Collections.Generic;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    [Serializable]
    public class PNavPolygon
    {
        public Fix64Vec2[] verts;
        public List<PNavEdge> edges = new List<PNavEdge>();
        public Fix64Vec2 centroid;
        public Fix64Vec2[] normal;
        public int vertsCount;
        public int index;

        public Fix64Vec3 Centroid3D
        {
            get
            {
                return new Fix64Vec3(centroid.x, Fix64.zero, centroid.y);
            }
        }

        public bool Equals(PNavPolygon other)
        {
            return index == other.index;
        }

        public PNavPolygon(Fix64Vec2[] vts, int vtsCount)
        {
            verts = vts;
            vertsCount = vtsCount;
            normal = new Fix64Vec2[vtsCount];
            ComputeCentroid();
            ComputeNormal();
        }

        public void ComputeCentroid()
        {
            Fix64Vec2 c = Fix64Vec2.zero;

            Fix64 area = Fix64.zero;

            Fix64Vec2 pRef = Fix64Vec2.zero;

            for (int i = 0; i < vertsCount; ++i)
            {
                pRef += verts[i];
            }

            pRef *= Fix64.one / (Fix64)vertsCount;

            Fix64 inv3 = Fix64.FromDivision(1, 3);

            for (int i = 0; i < vertsCount; ++i)
            {
                // Triangle vertices.
                Fix64Vec2 p1 = pRef;
                Fix64Vec2 p2 = verts[i];
                Fix64Vec2 p3 = i + 1 < vertsCount ? verts[i + 1] : verts[0];

                Fix64Vec2 e1 = p2 - p1;
                Fix64Vec2 e2 = p3 - p1;

                Fix64 D = e1.x * e2.y - e1.y * e2.x;

                Fix64 triangleArea = Fix64.half * D;
                area += triangleArea;

                // Area weighted centroid
                c += triangleArea * inv3 * (p1 + p2 + p3);
            }

            c *= Fix64.one / area;
            centroid = c;
        }

        public void ComputeNormal()
        {
            for (int i = 0; i < vertsCount; ++i)
            {
                int i1 = i;
                int i2 = i + 1 < vertsCount ? i + 1 : 0;
                Fix64Vec2 edge = verts[i2] - verts[i1];

                normal[i] = new Fix64Vec2(edge.y, -Fix64.one * edge.x);
                normal[i] = normal[i].normalized;
            }
        }

        public bool TestPoint(Fix64Vec2 point)
        {
            for (int i = 0; i < vertsCount; ++i)
            {
                Fix64 dot = Fix64Vec2.Dot(normal[i], point - verts[i]);

                if (dot > Fix64.zero)
                {
                    return false;
                }
            }

            return true;
        }
    }

}