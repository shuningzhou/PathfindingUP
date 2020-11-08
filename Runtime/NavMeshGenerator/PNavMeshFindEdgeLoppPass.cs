using Parallel;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshFindEdgeLoopPass
    {
        static PNavPoint[] points =
        {
            new PNavPoint(-1, -1), //bot left
            new PNavPoint(1, -1), //bot right
            new PNavPoint(1, 1), //top right
            new PNavPoint(-1, 1), //top left
            new PNavPoint(0, -1), //bot
            new PNavPoint(1, 0), //righ
            new PNavPoint(0, 1), //top
            new PNavPoint(-1,0), //left
    };

        static PNavPoint[] points2 =
        {
            new PNavPoint(-1,0), //left 0
            new PNavPoint(-1, 1), //top left 1
            new PNavPoint(0, 1), //top 2
            new PNavPoint(1, 1), //top right 3
            new PNavPoint(1, 0), //right 4
            new PNavPoint(1, -1), //bot right 5
            new PNavPoint(0, -1), //bot 6
            new PNavPoint(-1, -1), //bot left 7
    };

        static bool[] walkables =
        {
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false
    };

        public static void Process(PNavMesh pNavMesh)
        {
            using (new SProfiler("Finding Edge Loops"))
            {
                int maxX = pNavMesh.columns.GetLength(0) - 1;
                int maxZ = pNavMesh.columns.GetLength(1) - 1;
                PNavPoint pointMax = new PNavPoint(maxX, maxZ);

                FindEdgeLoops(pNavMesh.columns, pointMax, pNavMesh);
            }
        }

        static void FindEdgeLoops(PNavColumn[,] columns, PNavPoint pointMax, PNavMesh pNavMesh)
        {
            List<PNavIsland> islandsToRemove = new List<PNavIsland>();

            foreach (PNavIsland island in pNavMesh.islands)
            {
                int edgeLoopIndex = 0;
                int maxNodeCount = 0;
                PNavEdgeLoop edgeLoop = new PNavEdgeLoop();

                foreach (PNavNode node in island.nodes)
                {
                    if (node.walkable)
                    {
                        bool foundLoop = DetectEdgeLoop(node, edgeLoop, edgeLoopIndex, columns, pointMax);

                        if (foundLoop)
                        {
                            int nodeCount = edgeLoop.nodes.Count;
                            if (nodeCount > maxNodeCount)
                            {
                                maxNodeCount = nodeCount;
                                island.boundaryEdgeLoopIndex = edgeLoopIndex;
                            }
                            edgeLoopIndex++;
                            island.edgeLoops.Add(edgeLoop);
                            edgeLoop = new PNavEdgeLoop();
                        }
                    }
                }

                if (island.boundaryEdgeLoopIndex < 0)
                {
                    islandsToRemove.Add(island);
                }
            }

            foreach (PNavIsland island in islandsToRemove)
            {
                pNavMesh.islands.Remove(island);
            }
        }

        static bool DetectEdgeLoop(PNavNode node, PNavEdgeLoop edgeLoop, int edgeLoopIndex, PNavColumn[,] columns, PNavPoint pointMax)
        {
            bool foundLoop = false;

            if (node == null || !node.walkable || node.edgeLoopIndex >= 0)
            {
                return false;
            }

            bool isEdge = CheckIfNodeIsValidEdge(node, columns, pointMax);

            if (!isEdge)
            {
                return false;
            }

            Stack<PNavNode> stack = new Stack<PNavNode>();
            stack.Push(node);

            while (stack.Count > 0)
            {
                PNavNode n = stack.Pop();

                //possible to be added more than once
                if (n.edgeLoopIndex >= 0)
                {
                    continue;
                }

                edgeLoop.nodes.Add(n);
                n.edgeLoopIndex = edgeLoopIndex;
                foundLoop = true;

                for (int i = 0; i < 8; i++)
                {
                    PNavPoint pOut;
                    bool valid = PNavMeshHelper.GetPoint(n.point, pointMax, points[i], out pOut);

                    if (valid)
                    {
                        PNavNode nPush = columns[pOut.x, pOut.z].SurfaceNode();
                        if (nPush != null)
                        {
                            if (!nPush.walkable || nPush.edgeLoopIndex >= 0)
                            {
                                continue;
                            }

                            bool isEdge1 = CheckIfNodeIsValidEdge(nPush, columns, pointMax);

                            if (!isEdge1)
                            {
                                continue;
                            }

                            stack.Push(nPush);
                        }
                    }
                }
            }

            return foundLoop;
        }

        static bool CheckIfNodeIsValidEdge(PNavNode node, PNavColumn[,] columns, PNavPoint pointMax)
        {
            node.isEdge = false;

            node.edgeFlag = 0;

            for (int i = 0; i < 8; i++)
            {
                walkables[i] = false;

                PNavPoint pOut;
                bool valid = PNavMeshHelper.GetPoint(node.point, pointMax, points2[i], out pOut);

                if (valid)
                {
                    PNavNode n = columns[pOut.x, pOut.z].SurfaceNode();

                    if (n != null && n.walkable)
                    {

                        walkables[i] = true;
                        node.edgeFlag = (byte)(node.edgeFlag | 1 << i);
                    }
                }
            }

            if (walkables[0] != walkables[4]) //left and right
            {
                node.isEdge = true;

                // if top and bot are not walkable, not valid
                if (!walkables[2] && !walkables[6])
                {
                    node.isEdge = false;
                }
            }

            if (walkables[2] != walkables[6]) // top and bot
            {
                node.isEdge = true;

                // if left and right are not walkable, not valid
                if (!walkables[0] && !walkables[4])
                {
                    node.isEdge = false;
                }
            }

            // if all diagonal nodes are not walkable, not valid
            if (!walkables[1] && !walkables[3] && !walkables[5] && !walkables[7])
            {
                node.isEdge = false;
            }

            return node.isEdge;
        }

        static bool isWalkableNode(PNavPoint self, PNavPoint max, PNavPoint delta, PNavColumn[,] columns)
        {
            PNavPoint pOut;
            bool valid = PNavMeshHelper.GetPoint(self, max, delta, out pOut);

            if (valid)
            {
                PNavNode n = columns[pOut.x, pOut.z].SurfaceNode();

                valid = (n != null && n.walkable);
            }

            return valid;
        }
    }
}