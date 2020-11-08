using Parallel;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshBuildPolygonGraphPass
    {
        public static void Process(PNavMesh pNavMesh)
        {
            using (new SProfiler($"Build polygon graph"))
            {
                foreach (PNavIsland island in pNavMesh.islands)
                {
                    Dictionary<long, PNavEdge> edgeMap = new Dictionary<long, PNavEdge>();
                    PNavPolygonGraph graph = new PNavPolygonGraph();

                    int indiceRead = 0;
                    for (int p = 0; p < island.polygonCount; p++)
                    {
                        int ic = island.indiceCountsOfPolygons[p];

                        Fix64Vec2[] verts = new Fix64Vec2[ic];
                        int[] indices = new int[ic];

                        for (int indiceOfPolygon = 0; indiceOfPolygon < ic; indiceOfPolygon++)
                        {
                            int nodeIndex = island.indices[indiceRead];
                            PNavNode vertNode = island.nodes[nodeIndex];
                            verts[indiceOfPolygon] = new Fix64Vec2(vertNode.Center.x, vertNode.Center.z);
                            indices[indiceOfPolygon] = nodeIndex;
                            indiceRead++;
                        }

                        PNavPolygon polygon = new PNavPolygon(verts, ic);

                        //polygon index should match its index in graph.polygons list
                        polygon.index = p;
                        graph.AddPolygon(polygon);

                        BuildEdges(graph, indices, ic, polygon, edgeMap);
                    }

                    island.graph = graph;
                }
            }
        }

        static void BuildEdges(PNavPolygonGraph graph, int[] indices, int count, PNavPolygon polygon, Dictionary<long, PNavEdge> edgeMap)
        {
            int previous = 0;
            Fix64Vec2 previousVector2 = Fix64Vec2.zero;

            for (int i = 0; i < count; i++)
            {
                int current = indices[i];
                Fix64Vec2 currentVector2 = polygon.verts[i];

                if (i != 0)
                {
                    long key = KeyForIndices(previous, current);
                    if (edgeMap.ContainsKey(key))
                    {
                        PNavEdge edge = edgeMap[key];
                        edge.otherPolygonIndex = polygon.index;
                        edge.hasOther = true;
                        PNavPolygon self = graph.polygons[edge.selfPolygonIndex];
                        edge.distance = Fix64Vec2.Distance(self.centroid, polygon.centroid);

                        PNavEdge newEdge = new PNavEdge(edge);
                        newEdge.SwapDirection();

                        polygon.edges.Add(newEdge);
                    }
                    else
                    {
                        PNavEdge edge = new PNavEdge();
                        edge.selfPolygonIndex = polygon.index;
                        edge.width = Fix64Vec2.Distance(previousVector2, currentVector2);
                        edge.pointA = previousVector2;
                        edge.pointB = currentVector2;
                        edgeMap[key] = edge;
                        polygon.edges.Add(edge);
                    }
                }

                previous = current;
                previousVector2 = currentVector2;
            }

            int lastCurrent = indices[0];
            long lastKey = KeyForIndices(previous, lastCurrent);
            if (edgeMap.ContainsKey(lastKey))
            {
                PNavEdge edge = edgeMap[lastKey];
                edge.otherPolygonIndex = polygon.index;
                edge.hasOther = true;
                PNavPolygon self = graph.polygons[edge.selfPolygonIndex];
                edge.distance = Fix64Vec2.Distance(self.centroid, polygon.centroid);

                PNavEdge newEdge = new PNavEdge(edge);
                newEdge.SwapDirection();

                polygon.edges.Add(newEdge);
            }
            else
            {
                PNavEdge edge = new PNavEdge();
                edge.selfPolygonIndex = polygon.index;
                edge.width = Fix64Vec2.Distance(previousVector2, polygon.verts[0]);
                edge.pointA = previousVector2;
                edge.pointB = polygon.verts[0];
                edgeMap[lastKey] = edge;
                polygon.edges.Add(edge);
            }
        }

        static long KeyForIndices(int index1, int index2)
        {
            int small = 0;
            int large = 0;

            if (index1 > index2)
            {
                small = index2;
                large = index1;
            }
            else
            {
                small = index1;
                large = index2;
            }

            return (long)small * 10000000 + large;
        }
    }
}
