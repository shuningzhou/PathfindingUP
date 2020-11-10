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

        public PNavPolygon FindNearestPolygong(Fix64Vec2 point)
        {
            Fix64 min = Fix64.FromDivision(1000, 1);
            PNavPolygon minPolygon = null;
            foreach (PNavPolygon polygon in graph.polygons)
            {
                Fix64 dis = Fix64Vec2.Distance(polygon.centroid, point);
                if (dis < min)
                {
                    min = dis;
                    minPolygon = polygon;
                }
            }

            return minPolygon;
        }
    }
}