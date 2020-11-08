using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class NavMeshGenerator
    {
        PNavMesh _pNavMesh;

        public NavMeshGenerator(PNavMesh pNavMesh)
        {
            _pNavMesh = pNavMesh;
        }

        public void Generate()
        {
            PNavMeshFindAABBPass.Process(_pNavMesh);
            PNavMeshFindIslandPass.Process(_pNavMesh);
            PNavMeshFindEdgeLoopPass.Process(_pNavMesh);
            PNavMeshFindCornerPass.Process(_pNavMesh);
            PNavMeshTriangulationPass.Process(_pNavMesh);
            PNavMeshBuildPolygonGraphPass.Process(_pNavMesh);
        }
    }
}