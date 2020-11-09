using System;
using System.Collections.Generic;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public class NavMeshAStarNode : BaseAStarNode<PNavPolygon>
    {
        public Fix64Vec2 startPoint = Fix64Vec2.zero;
        //Fix64Vec2 nextStartPoint = Fix64Vec2.zero;
        public Fix64Vec2 endPoint = Fix64Vec2.zero;
        public bool isLastNode = false;

        public NavMeshAStarNode(PNavPolygon polygon) : base(polygon)
        {

        }

        public override Fix64 CalculateG(BaseAStarNode<PNavPolygon> other)
        {
            NavMeshAStarNode otherNode = (NavMeshAStarNode)other;

            Fix64 result = Fix64.zero;

            foreach (PNavEdge edge in other.UserObject.edges)
            {
                if (edge.hasOther && edge.otherPolygonIndex == UserObject.index)
                {
                    result = Fix64Vec2.DistanceToSegment(startPoint, edge.pointA, edge.pointB, out otherNode.startPoint);

                    if(otherNode.isLastNode)
                    {
                        result = result + Fix64Vec2.Distance(otherNode.endPoint, otherNode.startPoint);
                    }
                    //result = edge.distance;
                }
            }

            return result;
        }

        public override Fix64 CalculateH(BaseAStarNode<PNavPolygon> other)
        {
            NavMeshAStarNode otherNode = (NavMeshAStarNode)other;
            if(otherNode.isLastNode)
            {
                return Fix64Vec2.Distance(startPoint, otherNode.endPoint);
            }
            else
            {
                return Fix64Vec2.Distance(startPoint, otherNode.UserObject.centroid);
            }

            //Fix64 result = Fix64Vec2.Distance(UserObject.centroid, other.UserObject.centroid);
            //if (other)
            //return result;
        }

        public override bool Equals(BaseAStarNode<PNavPolygon> other)
        {
            return UserObject.index == other.UserObject.index;
        }

        public void Reset()
        {
            Parent = null;
            G = Fix64.zero;
            F = Fix64.zero;
            H = Fix64.zero;
            startPoint = Fix64Vec2.zero;
            endPoint = Fix64Vec2.zero;
            Index = 0;
            isLastNode = false;
        }
    }

    public class NavMeshAStart : BaseAStar<NavMeshAStarNode, PNavPolygon>
    {
        Dictionary<int, NavMeshAStarNode> nodeDictionary = new Dictionary<int, NavMeshAStarNode>();

        public void BuildNodeDictionary(List<PNavPolygon> polygons)
        {
            foreach (PNavPolygon polygon in polygons)
            {
                nodeDictionary[polygon.index] = new NavMeshAStarNode(polygon);
            }
        }

        public override int ChildrenOfNode(NavMeshAStarNode node, NavMeshAStarNode[] children)
        {
            int count = 0;

            foreach (PNavEdge edge in node.UserObject.edges)
            {
                if (edge.hasOther)
                {
                    int otherPolygonIndex = edge.otherPolygonIndex;
                    NavMeshAStarNode child = FindNode(otherPolygonIndex);
                    children[count] = child;
                    count++;
                }
            }

            return count;
        }

        public override int Compare(NavMeshAStarNode x, NavMeshAStarNode y)
        {
            if (x.F < y.F)
            {
                return -1;
            }
            else if (x.F > y.F)
            {
                return 1;
            }

            return 0;
        }

        public NavMeshAStarNode FindNode(int index)
        {
            if (nodeDictionary.ContainsKey(index))
            {
                return nodeDictionary[index];
            }

            return null;
        }

        public override void PrePathFinding()
        {
            foreach (var item in nodeDictionary.Values)
            {
                item.Reset();
            }
        }
    }
}