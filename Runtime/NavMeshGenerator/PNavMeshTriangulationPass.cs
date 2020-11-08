using Parallel;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshTriangulationPass
    {
        public static void Process(PNavMesh pNavMesh)
        {
            const int INDICE_COUNT = 1024;

            using (new SProfiler("Triangulation"))
            {
                int[] indices = new int[INDICE_COUNT];
                int[] indiceCounts = new int[INDICE_COUNT];
                Fix64Vec2[] verts = new Fix64Vec2[INDICE_COUNT];
                int[] indexes = new int[INDICE_COUNT];

                int index = 0;
                foreach (PNavIsland island in pNavMesh.islands)
                {
                    //build island with island boundary
                    if (island.boundaryEdgeLoopIndex < 0)
                    {
                        continue;
                    }

                    Debug.Log($"Generating Polygon graph for island={index}");
                    index++;
                    PNavEdgeLoop boundary = island.edgeLoops[island.boundaryEdgeLoopIndex];
                    int boundaryCornersCount = PrepareCornerVerts(boundary, island, verts, indexes);
                    PolyIsland polyIsland = Parallel3D.CreatePolyIsland(verts, indexes, boundaryCornersCount);

                    //add other edgeloops as holes
                    int edgeLoopIndex = -1;
                    foreach (PNavEdgeLoop edgeLoop in island.edgeLoops)
                    {
                        edgeLoopIndex++;

                        if (edgeLoopIndex == island.boundaryEdgeLoopIndex)
                        {
                            continue;
                        }

                        int holeCornersCount = PrepareCornerVerts(edgeLoop, island, verts, indexes);

                        Parallel3D.AddHolePolyIsland(verts, indexes, holeCornersCount, polyIsland);
                    }

                    int polygonCount = 0;
                    int totalIndicsCount = 0;

                    bool ok = Parallel3D.TriangulatePolyIsland(indices, indiceCounts, ref polygonCount, ref totalIndicsCount, 2, polyIsland);

                    int[] indicesCopy = new int[totalIndicsCount];
                    Array.Copy(indices, 0, indicesCopy, 0, totalIndicsCount);

                    int[] indiceCountsCopy = new int[polygonCount];
                    Array.Copy(indiceCounts, 0, indiceCountsCopy, 0, polygonCount);

                    island.indices = indicesCopy;
                    island.indiceCountsOfPolygons = indiceCountsCopy;
                    island.indicsCount = totalIndicsCount;
                    island.polygonCount = polygonCount;

                    Parallel3D.DestroyPolyIsland(polyIsland);
                }
            }
        }

        static int PrepareCornerVerts(PNavEdgeLoop edgeLoop, PNavIsland island, Fix64Vec2[] verts, int[] indexes)
        {
            List<PNavNode> corners = new List<PNavNode>();
            foreach (PNavNode node in edgeLoop.nodes)
            {
                if (node.isCorner)
                {
                    corners.Add(node);
                }
            }

            int cornersCount = corners.Count;

            for (int i = 0; i < cornersCount; i++)
            {
                PNavNode node = corners[i];
                Fix64Vec3 center = node.Center;
                verts[i] = new Fix64Vec2(center.x, center.z);
                indexes[i] = island.nodes.IndexOf(node);
            }

            bool isClockwise = IsClockwise(verts, cornersCount);

            if (!isClockwise)
            {
                InverseVerticesAndIndexes(verts, indexes, cornersCount);
            }

            return cornersCount;
        }

        static void InverseVerticesAndIndexes(Fix64Vec2[] verts, int[] indexes, int count)
        {
            for (int i = 0; i < count / 2; i++)
            {
                Fix64Vec2 tmpVert = verts[i];
                int tmpIndex = indexes[i];

                verts[i] = verts[count - i - 1];
                verts[count - i - 1] = tmpVert;

                indexes[i] = indexes[count - i - 1];
                indexes[count - i - 1] = tmpIndex;
            }
        }

        //https://stackoverflow.com/a/18472899
        static bool IsClockwise(Fix64Vec2[] vertices, int count)
        {
            Fix64 sum = Fix64.zero;
            for (int i = 0; i < count; i++)
            {
                Fix64Vec2 v1 = vertices[i];
                Fix64Vec2 v2 = vertices[(i + 1) % count];
                sum += (v2.x - v1.x) * (v2.y + v1.y);
            }
            return sum > Fix64.zero;
        }
    }
}