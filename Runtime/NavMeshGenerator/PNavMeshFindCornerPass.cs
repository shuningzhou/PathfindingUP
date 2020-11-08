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
                    PNavNode previousNode = edgeLoop.nodes[edgeLoop.nodes.Count - 1];
                    Vector2 direction = Vector2.zero;

                    foreach (PNavNode node in edgeLoop.nodes)
                    {
                        node.isCorner = false;

                        Vector2 newDirection = new Vector2(node.point.x - previousNode.point.x, node.point.z - previousNode.point.z).normalized;
                        float angle = Vector2.Angle(newDirection, direction);
                        previousNode.angle = angle;

                        if (angle >= 45)
                        {
                            previousNode.isCorner = true;
                        }

                        direction = newDirection;
                        previousNode = node;
                    }
                }
            }
        }
    }
}