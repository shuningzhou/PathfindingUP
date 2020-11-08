using Parallel;
using System;
using UnityEngine;

namespace Parallel.Pathfinding
{
    [Serializable]
    public class PNavMeshPath
    {
        public PNavMesh navMesh;
        public PNavIsland island;

        public ParallelNavMeshPathStatus Status;

        public int[] polygonIndexes;

        public const int MAX_CORNER_COUNT = 128;
        public int startIndex;

        public Fix64Vec3 Destination = Fix64Vec3.zero;
        public Fix64Vec2 Destination2D = Fix64Vec2.zero;
        public Fix64Vec3 nextCorner = Fix64Vec3.zero;

        public void UpdateCurrentPolygonIndex(Fix64Vec3 position)
        {

        }

        public bool PositionOnThePath(Fix64Vec2 position, ref int index)
        {
            bool positionIsOnThePath = false;
            int checkedPolygon = startIndex;
            PNavPolygon currentPolygon = null;

            while ((MAX_CORNER_COUNT - checkedPolygon) > 0)
            {
                int polygonIndex = polygonIndexes[checkedPolygon];
                PNavPolygon polygon = island.graph.polygons[polygonIndex];
                bool insidePolygon = polygon.TestPoint(position);
                if (insidePolygon)
                {
                    currentPolygon = polygon;
                    index = checkedPolygon;
                    positionIsOnThePath = true;
                    break;
                }
                checkedPolygon++;
            }

            return positionIsOnThePath;
        }

        public PNavEdge FindEdge(PNavPolygon currentPolygon, PNavPolygon nextPolygon, Fix64 width)
        {
            foreach (PNavEdge edge in nextPolygon.edges)
            {
                if (edge.hasOther && edge.otherPolygonIndex == currentPolygon.index)
                {
                    if (width > edge.width)
                    {
                        return null;
                    }
                    else
                    {
                        return edge;
                    }
                }
            }

            return null;
        }

        public int BoundaryCheckIndex(int index)
        {
            if (index < 0)
            {
                return startIndex;
            }

            if (index == MAX_CORNER_COUNT)
            {
                //reached last polygon, invalidate path
                return -1;
            }

            int nextIndex = index + 1;
            if (nextIndex >= MAX_CORNER_COUNT)
            {
                //reached final polygon
                return MAX_CORNER_COUNT;
            }

            return index;
        }

        public Fix64Vec2 FindNearestPointOnEdge(Fix64Vec2 testPosition, Fix64Vec2 pa, Fix64Vec2 pb, Fix64 width)
        {
            Fix64Vec2 pointOnEdge = Fix64Math.FindNearestPointOnLine(testPosition, pa, pb);
            Fix64 distanceToA = Fix64Vec2.Distance(pointOnEdge, pa);
            Fix64 edgeWidth = Fix64Vec2.Distance(pa, pb);
            Fix64 distanceToB = edgeWidth - distanceToA;
            Fix64Vec2 ba = pb - pa;
            ba = ba.normalized;
            Fix64 halfWidth = width / Fix64.two;
            if (distanceToA < halfWidth)
            {
                pointOnEdge = pa + ba * halfWidth;
            }
            else if (distanceToB < halfWidth)
            {
                pointOnEdge = pb - ba * halfWidth;
            }

            return pointOnEdge;
        }

        public bool NextCorner(Fix64Vec3 position, Fix64 width, ref Fix64Vec3 corner, ref int index)
        {
            //if corner not found stay at current position
            corner = position;

            if (Status != ParallelNavMeshPathStatus.Valid)
            {
                return false;
            }

            index = BoundaryCheckIndex(index);

            if (index == -1)
            {
                Status = ParallelNavMeshPathStatus.Invalid;
                return false;
            }

            Fix64Vec2 testPosition = new Fix64Vec2(position.x, position.z);

            Fix64Vec2 pa = Fix64Vec2.zero;
            Fix64Vec2 pb = Fix64Vec2.zero;
            bool pointABSet = false;

            if (polygonIndexes == null)
            {
                //destination polygon and start polygon is the same
                index = MAX_CORNER_COUNT;
            }

            while (index <= MAX_CORNER_COUNT)
            {
                if (index == MAX_CORNER_COUNT)
                {
                    //reached final polygon
                    if (pointABSet)
                    {
                        //check if destination is in the span of pa and pb
                        Fix64Vec2 des = new Fix64Vec2(Destination.x, Destination.z);
                        Fix64Vec2 vd = des - testPosition;
                        Fix64Vec2 va = pa - testPosition;
                        Fix64Vec2 vb = pb - testPosition;
                        bool desInSpan = Fix64Math.InSpan(vd, va, vb);
                        if (!desInSpan)
                        {
                            Fix64Vec2 pointOnEdge = FindNearestPointOnEdge(testPosition, pa, pb, width);
                            index--;
                            corner = new Fix64Vec3(pointOnEdge.x, position.y, pointOnEdge.y);
                            return true;
                        }
                    }

                    corner = Destination;
                    break;
                }

                int nextIndex = index + 1;
                int polygonIndex = polygonIndexes[index];
                PNavPolygon currentPolygon = island.graph.polygons[polygonIndex];
                int nextPolygonIndex = polygonIndexes[nextIndex];
                PNavPolygon nextPolygon = island.graph.polygons[nextPolygonIndex];

                PNavEdge edge = FindEdge(currentPolygon, nextPolygon, width);

                if (edge != null)
                {
                    if (!pointABSet)
                    {
                        pa = edge.pointA;
                        pb = edge.pointB;
                        pointABSet = true;
                    }
                    else
                    {
                        Fix64Vec2 va = pa - testPosition;
                        Fix64Vec2 vb = pb - testPosition;
                        Fix64 AXB = Fix64Vec2.Cross(va, vb);
                        Fix64 BXA = Fix64Vec2.Cross(vb, va);

                        Fix64Vec2 pa1 = edge.pointA;
                        Fix64Vec2 pb1 = edge.pointB;

                        Fix64Vec2 va1 = pa1 - testPosition;
                        Fix64Vec2 vb1 = pb1 - testPosition;
                        Fix64 A1XB1 = Fix64Vec2.Cross(va1, vb1);

                        bool sameDirection = A1XB1 * AXB > Fix64.zero;

                        bool va1InsideOfSpan = Fix64Math.InSpan(va1, va, vb);
                        bool vb1InsideOfSpan = Fix64Math.InSpan(vb1, va, vb);

                        if (va1InsideOfSpan)
                        {
                            if (sameDirection)
                            {
                                bool valid = IsValidEdgePoint(testPosition, pb, pa1, width);
                                if (valid)
                                {
                                    pa = pa1;
                                }
                                else
                                {
                                    va1InsideOfSpan = false;
                                }
                            }
                            else
                            {
                                bool valid = IsValidEdgePoint(testPosition, pa, pa1, width);
                                if (valid)
                                {
                                    pb = pa1;
                                }
                                else
                                {
                                    va1InsideOfSpan = false;
                                }
                            }
                        }


                        if (vb1InsideOfSpan)
                        {
                            if (sameDirection)
                            {
                                bool valid = IsValidEdgePoint(testPosition, pa, pb1, width);
                                if (valid)
                                {
                                    pb = pb1;
                                }
                                else
                                {
                                    vb1InsideOfSpan = false;
                                }
                            }
                            else
                            {
                                bool valid = IsValidEdgePoint(testPosition, pb, pb1, width);
                                if (valid)
                                {
                                    pa = pb1;
                                }
                                else
                                {
                                    vb1InsideOfSpan = false;
                                }
                            }
                        }

                        if (!va1InsideOfSpan && !vb1InsideOfSpan)
                        {
                            Fix64Vec2 pointOnEdge = FindNearestPointOnEdge(testPosition, pa, pb, width);

                            corner = new Fix64Vec3(pointOnEdge.x, position.y, pointOnEdge.y);
                            index--;
                            return true;
                        }
                    }
                }
                else
                {
                    break;
                }

                index = BoundaryCheckIndex(nextIndex);
            }

            return false;
        }

        public bool IsValidEdgePoint(Fix64Vec2 p, Fix64Vec2 pClose, Fix64Vec2 pFar, Fix64 width)
        {
            Fix64Vec2 vCloseP = pClose - p;
            Fix64Vec2 vFarP = pFar - p;
            Fix64 angle = Fix64Vec2.Angle(vCloseP, vFarP);

            //hypotenuse
            Fix64 h = Fix64Vec2.Distance(pClose, p);
            //adjacent
            Fix64 halfWidth = width / Fix64.two;
            Fix64 a = halfWidth;

            Fix64 minAngle = Fix64.two * Fix64.Asin(a / h) * Fix64.RadToDegree;

            if (minAngle > angle)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public bool NextCorner1(Fix64Vec3 position, Fix64 width, ref Fix64Vec3 corner, ref int index)
        {
            //if corner not found stay at current position
            corner = position;

            if (Status != ParallelNavMeshPathStatus.Valid)
            {
                return false;
            }

            if (index < 0)
            {
                index = startIndex;
            }

            if (index == MAX_CORNER_COUNT)
            {
                //reached last polygon, invalidate path
                index = -1;
                Status = ParallelNavMeshPathStatus.Invalid;
                return false;
            }

            Fix64Vec2 testPosition = new Fix64Vec2(position.x, position.z);

            int polygonIndex = polygonIndexes[index];
            PNavPolygon currentPolygon = island.graph.polygons[polygonIndex];

            int nextIndex = index + 1;
            if (nextIndex >= MAX_CORNER_COUNT)
            {
                //reached final polygon
                index = MAX_CORNER_COUNT;
                corner = Destination;
                return false;
            }

            int nextPolygonIndex = polygonIndexes[nextIndex];
            PNavPolygon nextPolygon = island.graph.polygons[nextPolygonIndex];
            PNavEdge edge = FindEdge(currentPolygon, nextPolygon, width);

            if (edge != null)
            {
                Fix64Vec2 pointOnEdge = Fix64Math.FindNearestPointOnLine(testPosition, edge.pointA, edge.pointB);
                Fix64 distanceToA = Fix64Vec2.Distance(pointOnEdge, edge.pointA);
                Fix64 distanceToB = edge.width - distanceToA;
                Fix64Vec2 ba = edge.pointB - edge.pointA;
                ba = ba.normalized;
                Fix64 halfWidth = width / Fix64.two;
                if (distanceToA < halfWidth)
                {
                    pointOnEdge = edge.pointA + ba * halfWidth;
                }
                else if (distanceToB < halfWidth)
                {
                    pointOnEdge = edge.pointB - ba * halfWidth;
                }

                corner = new Fix64Vec3(pointOnEdge.x, position.y, pointOnEdge.y);
                index = nextIndex;
                return true;
            }

            return false;
        }

        public void GetCorners(ref int count, Fix64Vec3[] corners)
        {

        }
    }
}