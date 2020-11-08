using Parallel;
using UnityEditor;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshDebugDraw
    {
        public static void DrawPolygonGraph(PNavMesh pNavMesh)
        {
            Color c = Handles.color;
            foreach (PNavIsland island in pNavMesh.islands)
            {
                int index = 0;
                foreach (PNavPolygon polygon in island.graph.polygons)
                {
                    Handles.color = Color.green;
                    Handles.SphereHandleCap(0, (Vector3)polygon.Centroid3D, Quaternion.identity, 0.1f, EventType.Repaint);

                    Handles.Label((Vector3)polygon.Centroid3D + Vector3.forward * 0.5f, index.ToString());

                    foreach (PNavEdge edge in polygon.edges)
                    {
                        if (edge.hasOther)
                        {
                            PNavPolygon self = island.graph.polygons[edge.selfPolygonIndex];
                            PNavPolygon other = island.graph.polygons[edge.otherPolygonIndex];
                            Handles.DrawLine((Vector3)self.Centroid3D, (Vector3)other.Centroid3D);
                        }
                    }

                    index++;
                }
            }
            Handles.color = c;
        }

        public static void DrawAABB(PNavMesh pNavMesh)
        {
            if (pNavMesh.columns != null)
            {
                for (int x = 0; x < pNavMesh.columns.GetLength(0); x++)
                {
                    //if (x * pNavMesh.gridSize.x < 95)
                    {
                        //continue;
                    }

                    for (int z = 0; z < pNavMesh.columns.GetLength(1); z++)
                    {
                        PNavColumn column = pNavMesh.columns[x, z];

                        for (int y = 0; y < column.nodes.Length; y++)
                        {
                            int objectCount = column.nodes[y].objectCount;
                            Fix64Vec3 l = column.nodes[y].lower;
                            Fix64Vec3 u = column.nodes[y].upper;

                            //if (l.x * pNavMesh.gridSize.x < 30)
                            {
                                //continue;
                            }

                            if (objectCount > 0)
                            {
                                SceneDebugDraw.DrawAABB(l, u, Color.green);
                            }
                            else
                            {
                                //DrawAABB(l, u, Color.green);
                            }
                        }
                    }
                }
            }
        }

        static Color[] colors =
        {
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
        new Color(Random.Range(0.5F,1F), Random.Range(0.5F, 1F), Random.Range(0.5F, 1F)),
    };

        public static void DrawEdgeLoop(PNavMesh pNavMesh)
        {
            Color c = Handles.color;
            foreach (PNavIsland island in pNavMesh.islands)
            {
                int loopIndex = 0;
                foreach (PNavEdgeLoop loop in island.edgeLoops)
                {
                    loopIndex++;
                    Handles.color = colors[loopIndex];

                    foreach (PNavNode node in loop.nodes)
                    {
                        Handles.SphereHandleCap(0, (Vector3)node.Center, Quaternion.identity, 0.1f, EventType.Repaint);
                    }
                }
            }
            Handles.color = c;
        }

        public static void DrawEdgeGap(PNavMesh pNavMesh)
        {
            foreach (PNavIsland island in pNavMesh.islands)
            {

                foreach (PNavNode node in island.nodes)
                {
                    Color c = Handles.color;

                    if (node.IsInner)
                    {
                        if (node.walkable)
                        {

                        }
                        else
                        {
                            Handles.color = Color.yellow;
                            Handles.SphereHandleCap(0, (Vector3)node.Center, Quaternion.identity, 0.1f, EventType.Repaint);
                        }

                        continue;
                    }

                    Handles.color = c;
                }
            }
        }

        public static void DrawEdge(PNavMesh pNavMesh)
        {
            float arrowSize = (float)pNavMesh.gridSize.x / 2;
            foreach (PNavIsland island in pNavMesh.islands)
            {
                foreach (PNavNode node in island.nodes)
                {
                    Color c = Handles.color;

                    if (node.IsInner)
                    {

                    }
                    else
                    {
                        Handles.color = Color.red;
                        Handles.SphereHandleCap(0, (Vector3)node.Center, Quaternion.identity, 0.1f, EventType.Repaint);

                        if (node.IsLeftEdge)
                        {
                            Handles.DrawLine((Vector3)node.LeftCenter - arrowSize * Vector3.forward, (Vector3)node.LeftCenter + arrowSize * Vector3.forward);
                        }

                        if (node.IsRghtEdge)
                        {
                            Handles.DrawLine((Vector3)node.RightCenter - arrowSize * Vector3.forward, (Vector3)node.RightCenter + arrowSize * Vector3.forward);
                        }

                        if (node.IsFrontEdge)
                        {
                            Handles.DrawLine((Vector3)node.FrontCenter - arrowSize * Vector3.right, (Vector3)node.FrontCenter + arrowSize * Vector3.right);
                        }

                        if (node.IsBackEdge)
                        {
                            Handles.DrawLine((Vector3)node.BackCenter - arrowSize * Vector3.right, (Vector3)node.BackCenter + arrowSize * Vector3.right);
                        }
                    }

                    Handles.color = c;
                }
            }
        }

        public static void DrawWalkable(PNavMesh pNavMesh)
        {
            foreach (PNavIsland island in pNavMesh.islands)
            {
                foreach (PNavNode node in island.nodes)
                {
                    Color c = Handles.color;

                    if (node.IsInner)
                    {
                        if (node.walkable)
                        {
                            if (node.isCorner)
                            {
                                Handles.color = Color.cyan;
                                Handles.SphereHandleCap(0, (Vector3)node.Center, Quaternion.identity, 0.1f, EventType.Repaint);
                            }
                            else if (node.isEdge)
                            {
                                Handles.color = Color.magenta;
                                Handles.SphereHandleCap(0, (Vector3)node.Center, Quaternion.identity, 0.1f, EventType.Repaint);
                            }
                            else
                            {
                                Handles.color = Color.green;
                                Handles.SphereHandleCap(0, (Vector3)node.Center, Quaternion.identity, 0.1f, EventType.Repaint);
                            }
                        }
                    }

                    Handles.color = c;
                }
            }
        }

        public static void DrawCorner(PNavMesh pNavMesh)
        {
            foreach (PNavIsland island in pNavMesh.islands)
            {
                foreach (PNavNode node in island.nodes)
                {
                    Color c = Handles.color;

                    if (node.IsInner)
                    {
                        if (node.walkable)
                        {
                            if (node.isCorner)
                            {
                                Handles.color = Color.cyan;
                                Handles.SphereHandleCap(0, (Vector3)node.Center, Quaternion.identity, 0.1f, EventType.Repaint);
                            }
                        }
                    }

                    Handles.color = c;
                }
            }
        }

        public static void DrawPolygon(PNavMesh pNavMesh)
        {
            foreach (PNavIsland island in pNavMesh.islands)
            {
                Fix64Vec3[] verts = new Fix64Vec3[1024];

                int indiceRead = 0;
                for (int p = 0; p < island.polygonCount; p++)
                {
                    int ic = island.indiceCountsOfPolygons[p];
                    for (int indiceOfPolygon = 0; indiceOfPolygon < ic; indiceOfPolygon++)
                    {
                        int nodeIndex = island.indices[indiceRead];
                        PNavNode vertNode = island.nodes[nodeIndex];
                        verts[indiceOfPolygon] = vertNode.Center;
                        indiceRead++;
                    }

                    DrawPolygon(verts, ic, Color.cyan);
                }
            }
        }

        static void DrawPolygon(Fix64Vec3[] verts, int count, Color color)
        {
            Color c = Handles.color;

            Handles.color = color;

            Fix64Vec3 previous = Fix64Vec3.zero;

            for (int i = 0; i < count; i++)
            {
                Fix64Vec3 current = verts[i];
                if (i != 0)
                {
                    Handles.DrawLine((Vector3)previous, (Vector3)current);
                }

                previous = current;
            }

            Handles.DrawLine((Vector3)previous, (Vector3)verts[0]);

            Handles.color = c;
        }
    }

}