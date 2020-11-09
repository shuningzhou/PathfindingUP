using System.Collections.Generic;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public class PNavMesh : ScriptableObject
    {
        public bool saved = false;
        public Fix64Vec3 gridSize = new Fix64Vec3(Fix64.quarter, Fix64.quarter, Fix64.quarter);
        
        public Fix64Vec3 worldOrigin = Fix64Vec3.zero;

        public Fix64Vec3 worldSize = new Fix64Vec3(Fix64.FromDivision(10, 1), Fix64.FromDivision(10, 1), Fix64.FromDivision(10, 1));

        public int verticalDrop = 1;
        public int edgeGap = 1;

        public PNavColumn[,] columns;

        public List<PNavIsland> islands;

        public int xCount;
        public int yCount;
        public int zCount;

        public void GetAABB(int x, int y, int z, out Fix64Vec3 lower, out Fix64Vec3 upper)
        {
            Fix64 lowerX = worldOrigin.x;
            Fix64 lowerY = worldOrigin.y;
            Fix64 lowerZ = worldOrigin.z;

            lower = new Fix64Vec3(
                lowerX + (Fix64)x * gridSize.x,
                lowerY + (Fix64)y * gridSize.y,
                lowerZ + (Fix64)z * gridSize.z);

            upper = new Fix64Vec3(
                lowerX + ((Fix64)x + Fix64.one) * gridSize.x,
                lowerY + ((Fix64)y + Fix64.one) * gridSize.y,
                lowerZ + ((Fix64)z + Fix64.one) * gridSize.z);
        }

        public void Reset()
        {
            columns = null;
            if (islands != null)
            {
                islands.Clear();
            }

            saved = false;
        }

        public void Save()
        {
            columns = null;
            foreach(PNavIsland island in islands)
            {
                island.Save();
            }

            saved = true;
        }
    }
}