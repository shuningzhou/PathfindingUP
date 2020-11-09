using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Parallel.Pathfinding
{
    public enum ParallelNavColumnType
    {
        Empty = 0,
        Walkable = 1
    }

    public enum ParallelNavMeshPathStatus
    {
        Invalid = 0,
        Valid = 1
    }

    public enum ParallelNavIslandNodeType
    {
        Inner = 0,
        LeftEdge = 1,
        RightEdge = 2,
        TopEdge = 4,
        BottomEdge = 8,
        CornerEdge = 16
    }
}