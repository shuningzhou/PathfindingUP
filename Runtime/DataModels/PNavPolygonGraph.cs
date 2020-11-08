using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Parallel.Pathfinding
{
    [Serializable]
    public class PNavPolygonGraph
    {
        public List<PNavPolygon> polygons = new List<PNavPolygon>();

        public void AddPolygon(PNavPolygon polygon)
        {
            polygons.Add(polygon);
        }
    }
}