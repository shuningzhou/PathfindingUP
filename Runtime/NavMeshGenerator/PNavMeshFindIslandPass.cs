using Parallel;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshFindIslandPass
    {
        public static void Process(PNavMesh pNavMesh)
        {
            using (new SProfiler("Finding Islands"))
            {
                // find islands
                List<PNavIsland> islands = new List<PNavIsland>();
                int maxX = pNavMesh.columns.GetLength(0) - 1;
                int maxZ = pNavMesh.columns.GetLength(1) - 1;
                PNavPoint pointMax = new PNavPoint(maxX, maxZ);
                int islandIndex = 0;
                PNavIsland island = new PNavIsland();

                for (int x = 0; x < maxX; x++)
                {
                    for (int z = 0; z < maxZ; z++)
                    {
                        PNavPoint point = new PNavPoint(x, z);
                        bool foundIsland = DetectIsland(pNavMesh.columns, point, pointMax, 1, pNavMesh.verticalDrop, islandIndex, island);
                        if (foundIsland)
                        {
                            islandIndex++;
                            islands.Add(island);
                            PNavMeshHelper.DetectEdgeCorner(pNavMesh.columns, pointMax, island);
                            PNavMeshHelper.ApplyEdgeGap(pNavMesh.columns, pointMax, island, pNavMesh.edgeGap);

                            island = new PNavIsland();
                        }
                    }
                }

                pNavMesh.islands = islands;
                Debug.Log($"Found {islands.Count} islands");
            }
        }


        static bool DetectIsland(PNavColumn[,] columns, PNavPoint point, PNavPoint pointMax, int edgeGap, int verticalDrop, int islandIndex, PNavIsland island)
        {
            PNavColumn column = columns[point.x, point.z];

            if (column.type != ParallelNavColumnType.Walkable)
            {
                return false;
            }

            PNavNode surfaceNode = column.SurfaceNode();

            if (surfaceNode == null)
            {
                return false;
            }

            //already checked
            if (surfaceNode.islandIndex >= 0)
            {
                return false;
            }

            surfaceNode.islandIndex = islandIndex;

            island.nodes.Add(surfaceNode);

            Queue<PNavPoint> queue = new Queue<PNavPoint>();

            queue.Enqueue(point);

            while (queue.Count > 0)
            {
                PNavPoint p = queue.Dequeue();

                PNavColumn c = columns[p.x, p.z];

                //check top edge
                PNavPoint pTopOut;

                bool addedTop = PNavMeshHelper.AddTop(columns, p, out pTopOut, pointMax, verticalDrop, edgeGap, islandIndex, island);

                if (addedTop)
                {
                    queue.Enqueue(pTopOut);
                }

                //check bottom edge
                PNavPoint pBottomOut;
                bool addedBottom = PNavMeshHelper.AddBottom(columns, p, out pBottomOut, pointMax, verticalDrop, edgeGap, islandIndex, island);

                if (addedBottom)
                {
                    queue.Enqueue(pBottomOut);
                }

                //search left
                PNavPoint left = p;

                while (true)
                {
                    PNavPoint pLeftOut;

                    bool added = PNavMeshHelper.AddLeft(columns, left, out pLeftOut, pointMax, verticalDrop, edgeGap, islandIndex, island, queue);

                    if (!added)
                    {
                        break;
                    }
                    else
                    {
                        left = pLeftOut;
                    }
                }

                //search right

                PNavPoint right = p;

                while (true)
                {
                    PNavPoint pRightOut;

                    bool added = PNavMeshHelper.AddRight(columns, right, out pRightOut, pointMax, verticalDrop, edgeGap, islandIndex, island, queue);

                    if (!added)
                    {
                        break;
                    }
                    else
                    {
                        right = pRightOut;
                    }
                }
            }

            return true;
        }
    }
}