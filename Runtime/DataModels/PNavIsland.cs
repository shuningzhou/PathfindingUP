using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Parallel.Pathfinding
{
    [Serializable]
    public class PNavIsland
    {
        public List<PNavNode> nodes = new List<PNavNode>();
        public int boundaryEdgeLoopIndex = -1;
        public List<PNavEdgeLoop> edgeLoops = new List<PNavEdgeLoop>();
        public List<PNavNode> corners = new List<PNavNode>();

        public int[] indices;
        public int[] indiceCountsOfPolygons;
        public int polygonCount;
        public int indicsCount;

        public PNavPolygonGraph graph;

        public void Save()
        {
            nodes = null;
            edgeLoops = null;
            corners = null;
            indices = null;
            indiceCountsOfPolygons = null;
        }
    }
}