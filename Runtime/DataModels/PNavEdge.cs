using System;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    [Serializable]
    public class PNavEdge
    {
        public Fix64Vec2 pointA;
        public Fix64Vec2 pointB;
        public int selfPolygonIndex;
        public int otherPolygonIndex;
        //public PNavPolygon self;
        //public PNavPolygon other;
        public Fix64 distance;
        public Fix64 width;
        public bool hasOther;

        public PNavEdge()
        {

        }

        public PNavEdge(PNavEdge edge)
        {
            selfPolygonIndex = edge.selfPolygonIndex;
            otherPolygonIndex = edge.otherPolygonIndex;
            distance = edge.distance;
            width = edge.width;
            hasOther = edge.hasOther;
            pointA = edge.pointA;
            pointB = edge.pointB;
        }

        public void SwapDirection()
        {
            int temp = selfPolygonIndex;
            selfPolygonIndex = otherPolygonIndex;
            otherPolygonIndex = temp;
        }
    }

}