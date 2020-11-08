using System.Collections.Generic;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public class PNavMesh : ScriptableObject
    {
        public bool saved = false;
        public Fix64Vec3 gridSize = new Fix64Vec3(Fix64.quarter, Fix64.quarter, Fix64.quarter);
        public Fix64 floorLevel = Fix64.zero;
        public Fix64 ceilingLevel = Fix64.FromDivision(10, 1);
        public int verticalDrop = 1;
        public Fix64Vec3 worldLowerBound;
        public Fix64Vec3 worldUpperBound;

        public PNavColumn[,] columns;

        public List<PNavIsland> islands;

        public int xCount;
        public int yCount;
        public int zCount;

        public void GetAABB(int x, int y, int z, out Fix64Vec3 lower, out Fix64Vec3 upper)
        {
            lower = new Fix64Vec3(
                worldLowerBound.x + (Fix64)x * gridSize.x,
                worldLowerBound.y + (Fix64)y * gridSize.y,
                worldLowerBound.z + (Fix64)z * gridSize.z);

            upper = new Fix64Vec3(
                worldLowerBound.x + ((Fix64)x + Fix64.one) * gridSize.x,
                worldLowerBound.y + ((Fix64)y + Fix64.one) * gridSize.y,
                worldLowerBound.z + ((Fix64)z + Fix64.one) * gridSize.z);
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