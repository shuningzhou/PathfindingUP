using Parallel;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshRemoveNarrowPathPass
    {
        static PNavPoint[] points =
        {
            new PNavPoint(-1,0), //left 0
            new PNavPoint(0, 1), //top 1
            new PNavPoint(1, 0), //right 2
            new PNavPoint(0, -1), //bot 3
    };

        public static void Process(PNavMesh pNavMesh)
        {
            using (new SProfiler($"Remove Narrow Path"))
            {
                foreach (PNavIsland island in pNavMesh.islands)
                {
                    foreach (PNavNode node in island.nodes)
                    {

                    }
                }
            }
        }
    }
}