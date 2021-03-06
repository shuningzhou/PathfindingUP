﻿using System.Collections.Generic;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public class PNavMeshManager : MonoBehaviour
    {
        public PNavMesh navMesh;

        public Dictionary<PNavIsland, NavMeshAStart> astartDictionary = new Dictionary<PNavIsland, NavMeshAStart>();
        // Start is called before the first frame update
        void Start()
        {
            foreach (PNavIsland island in navMesh.islands)
            {
                NavMeshAStart astar = new NavMeshAStart();
                astar.BuildNodeDictionary(island.graph.polygons);
                astartDictionary[island] = astar;
            }
        }

        public PNavMeshPath CalculatePath(Fix64Vec3 start, Fix64Vec3 end)
        {
            Fix64Vec2 startPosition = new Fix64Vec2(start.x, start.z);
            Fix64Vec2 endPosition = new Fix64Vec2(end.x, end.z);
            PNavPolygon startPolygon = null;
            PNavPolygon endPolygon = null;

            bool sameIsland = false;
            NavMeshAStart astart = null;

            PNavMeshPath result = new PNavMeshPath();
            result.Destination = end;
            result.Destination2D = endPosition;
            result.Status = ParallelNavMeshPathStatus.Invalid;
            result.navMesh = navMesh;

            PNavIsland endIsland = null;
            PNavIsland startIsland = null;

            foreach (PNavIsland island in navMesh.islands)
            {
                bool foundStart = false;
                bool foundEnd = false;

                foreach (PNavPolygon polygon in island.graph.polygons)
                {
                    bool isStart = polygon.TestPoint(startPosition);
                    bool isEnd = polygon.TestPoint(endPosition);

                    if (isStart && isEnd)
                    {
                        result.Status = ParallelNavMeshPathStatus.Valid;
                        result.startIndex = -1;
                        return result;
                    }

                    if (isStart)
                    {
                        startPolygon = polygon;
                        startIsland = island;
                        foundStart = true;
                    }

                    if (isEnd)
                    {
                        endPolygon = polygon;
                        endIsland = island;
                        foundEnd = true;
                    }
                }

                if (foundStart && foundEnd)
                {
                    sameIsland = true;
                    astart = astartDictionary[island];
                    result.island = island;
                    break;
                }

                if (sameIsland)
                {
                    break;
                }
            }

            if(startPolygon != null && endPolygon == null)
            {
                //we find the polygon that is closest to the end position in the start position island
                sameIsland = true;
                astart = astartDictionary[startIsland];
                result.island = startIsland;
                endPolygon = startIsland.FindNearestPolygong(endPosition);
            }
            else if(startPolygon == null && endPolygon != null)
            {
                //we find the polygon that is closest to the start position in the end position island
                sameIsland = true;
                astart = astartDictionary[endIsland];
                result.island = endIsland;
                startPolygon = endIsland.FindNearestPolygong(startPosition);
            }
            else if(startPolygon == null && endPolygon == null)
            {
                //we find the polygon that is closest to the start position
                //we then find the polygon that is closest to the end position in the same island
                Fix64 minStart = Fix64.FromDivision(1000, 1);
                PNavPolygon minStartPolygon = null;
                PNavIsland minStartIsland = null;

                foreach (PNavIsland island in navMesh.islands)
                {
                    foreach (PNavPolygon polygon in island.graph.polygons)
                    {
                        Fix64 dis = Fix64Vec2.Distance(polygon.centroid, startPosition);
                        if(dis < minStart)
                        {
                            minStart = dis;
                            minStartPolygon = polygon;
                            minStartIsland = island;
                        }
                    }
                }


                sameIsland = true;
                astart = astartDictionary[minStartIsland];
                result.island = minStartIsland;
                startPolygon = minStartPolygon;

                endPolygon = minStartIsland.FindNearestPolygong(endPosition);
            }

            if (sameIsland)
            {
                if (astart != null && startPolygon != null && endPolygon != null)
                {
                    NavMeshAStarNode startNode = astart.FindNode(startPolygon.index);
                    NavMeshAStarNode endNode = astart.FindNode(endPolygon.index);
                    NavMeshAStarNode lastNode = null;

                    using (new SProfiler("Pathfinding"))
                    {
                        astart.PrePathFinding();

                        startNode.startPoint = startPosition;
                        endNode.isLastNode = true;
                        endNode.endPoint = endPosition;

                        lastNode = astart.FindPath(startNode, endNode);
                    }

                    result.polygonIndexes = new int[PNavMeshPath.MAX_CORNER_COUNT];

                    int startIndex = 127;

                    while (lastNode != null)
                    {
                        result.polygonIndexes[startIndex] = lastNode.UserObject.index;
                        lastNode = (NavMeshAStarNode)lastNode.Parent;
                        startIndex--;
                    }

                    result.startIndex = startIndex + 1;
                    result.Status = ParallelNavMeshPathStatus.Valid;
                }
            }

            return result;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyUp(KeyCode.Return))
            {

            }
        }
    }
}
