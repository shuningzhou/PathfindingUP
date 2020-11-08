using System;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public struct PNavPoint
    {
        public int x;
        public int z;

        public PNavPoint(int X, int Z)
        {
            x = X;
            z = Z;
        }
    }

    [Serializable]
    public class PNavNode
    {
        public Fix64Vec3 lower;
        public Fix64Vec3 upper;
        public int objectCount;
        public int islandIndex;
        public int type;
        public bool walkable = true;
        public PNavPoint point;
        public bool isEdge = false;
        public bool isCorner = false;
        public float angle = 0;
        public int edgeLoopIndex = -1;
        public byte edgeFlag = 0;

        public Fix64Vec3 Center
        {
            get
            {
                return Fix64.half * (lower + upper);
            }
        }

        public Fix64Vec3 LeftCenter
        {
            get
            {
                return new Fix64Vec3(lower.x, Fix64.half * (lower.y + upper.y), Fix64.half * (lower.z + upper.z));
            }
        }

        public Fix64Vec3 RightCenter
        {
            get
            {
                return new Fix64Vec3(upper.x, Fix64.half * (lower.y + upper.y), Fix64.half * (lower.z + upper.z));
            }
        }

        public Fix64Vec3 FrontCenter
        {
            get
            {
                return new Fix64Vec3(Fix64.half * (lower.x + upper.x), Fix64.half * (lower.y + upper.y), upper.z);
            }
        }

        public Fix64Vec3 BackCenter
        {
            get
            {
                return new Fix64Vec3(Fix64.half * (lower.x + upper.x), Fix64.half * (lower.y + upper.y), lower.z);
            }
        }

        public bool IsInner
        {
            get
            {
                return type == (int)ParallelNavIslandNodeType.Inner;
            }
        }

        public bool IsLeftEdge
        {
            get
            {
                return (type & (int)ParallelNavIslandNodeType.LeftEdge) > 0;
            }
        }

        public bool IsRghtEdge
        {
            get
            {
                return (type & (int)ParallelNavIslandNodeType.RightEdge) > 0;
            }
        }

        public bool IsFrontEdge
        {
            get
            {
                return (type & (int)ParallelNavIslandNodeType.TopEdge) > 0;
            }
        }

        public bool IsBackEdge
        {
            get
            {
                return (type & (int)ParallelNavIslandNodeType.BottomEdge) > 0;
            }
        }
    }
}