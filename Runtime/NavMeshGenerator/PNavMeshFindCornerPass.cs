using Parallel;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshFindCornerPass
    {
        public static void Process(PNavMesh pNavMesh)
        {
            foreach (PNavIsland island in pNavMesh.islands)
            {
                foreach (PNavEdgeLoop edgeLoop in island.edgeLoops)
                {
                    PNavNode previousNode = null;
                    Vector2 direction = Vector2.zero;
                    int currentEdgeCount = 0;
                    bool skipCorner = false;
                    foreach (PNavNode node in edgeLoop.nodes)
                    {
                        node.isCorner = false;
                        if (currentEdgeCount == 0)
                        {
                            previousNode = node;
                            node.isCorner = true;
                            skipCorner = true;
                            node.angle = 0;
                            currentEdgeCount++;
                            island.corners.Add(node);
                        }
                        else if (currentEdgeCount == 1)
                        {
                            direction = new Vector2(node.point.x - previousNode.point.x, node.point.z - previousNode.point.z).normalized;
                            previousNode = node;
                            node.angle = 0;
                            currentEdgeCount++;
                        }
                        else
                        {
                            Vector2 newDirection = new Vector2(node.point.x - previousNode.point.x, node.point.z - previousNode.point.z);
                            float angle = Vector2.Angle(newDirection, direction);

                            if (angle >= 45 && !skipCorner)
                            {
                                //end current edge
                                previousNode.angle = angle;
                                previousNode.isCorner = true;
                                island.corners.Add(previousNode);

                                //new edge
                                direction = newDirection;
                                previousNode = node;
                                node.angle = 0;
                                currentEdgeCount = 2;
                                skipCorner = true;
                            }
                            else
                            {
                                //add to current edge
                                previousNode.angle = angle;
                                direction = (0.2f * newDirection + direction).normalized;
                                previousNode = node;
                                currentEdgeCount++;
                                skipCorner = false;
                            }
                        }
                    }
                }
            }
        }
    }
}