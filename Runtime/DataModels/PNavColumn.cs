using System;
using UnityEngine;

namespace Parallel.Pathfinding
{
    [Serializable]
    public class PNavColumn
    {
        public ParallelNavColumnType type;
        public PNavNode[] nodes;
        public int[] surfaceNodeIndexes;

        public PNavNode SurfaceNode()
        {
            if (type != ParallelNavColumnType.Walkable)
            {
                return null;
            }
            else
            {
                int sIndex = surfaceNodeIndexes[0];
                return nodes[sIndex];
            }
        }

        public bool IsConnected(PNavColumn c, int verticalDrop)
        {
            if (type == ParallelNavColumnType.Walkable && c.type == ParallelNavColumnType.Walkable)
            {
                int surfaceNodeIndex = c.surfaceNodeIndexes[0];

                if (Mathf.Abs(surfaceNodeIndexes[0] - surfaceNodeIndex) <= verticalDrop)
                {
                    return true;
                }
            }

            return false;
        }
    }
}
