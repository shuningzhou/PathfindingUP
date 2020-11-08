using System;
using System.Collections.Generic;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public class NavMeshAStarNode : BaseAStarNode<PNavPolygon>
    {
        public NavMeshAStarNode(PNavPolygon polygon) : base(polygon)
        {

        }

        public override bool Equals(PNavPolygon other)
        {
            return UserObject.index == other.index;
        }

        public override Fix64 CalculateG(PNavPolygon other)
        {
            Fix64 result = Fix64.zero;

            foreach (PNavEdge edge in other.edges)
            {
                if (edge.hasOther && edge.otherPolygonIndex == UserObject.index)
                {
                    result = edge.distance;
                }
            }

            return result;
        }

        public override Fix64 CalculateH(PNavPolygon other)
        {
            Fix64 result = Fix64Vec2.Distance(UserObject.centroid, other.centroid);
            return result;
        }

        public void Reset()
        {
            Parent = null;
            G = Fix64.zero;
            F = Fix64.zero;
            H = Fix64.zero;
            Index = 0;
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