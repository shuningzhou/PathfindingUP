using Parallel;
using System;
using UnityEngine;

namespace Parallel.Pathfinding
{
    public class PNavMeshFindAABBPass
    {
        public static void Process(PNavMesh pNavMesh)
        {
            ParallelRigidbody3D[] pRigidbody3Ds = GameObject.FindObjectsOfType<ParallelRigidbody3D>();

            using (new SProfiler($"Insert static bodies"))
            {
                //insert static bodies
                foreach (ParallelRigidbody3D rigidbody3D in pRigidbody3Ds)
                {
                    if (rigidbody3D.bodyType != BodyType.Static)
                    {
                        continue;
                    }

                    rigidbody3D.AddToWorldForPathFinding();
                }
            }

            // get world size
            Fix64Vec3 lower = Fix64Vec3.zero;
            Fix64Vec3 upper = Fix64Vec3.zero;
            Parallel3D.GetWorldSize(ref lower, ref upper);

            lower = new Fix64Vec3(
                lower.x,
                Fix64Math.Max(pNavMesh.floorLevel, lower.y),
                lower.z);

            upper = new Fix64Vec3(
                upper.x,
                Fix64Math.Min(pNavMesh.ceilingLevel, upper.y),
                upper.z);

            pNavMesh.worldLowerBound = lower;
            pNavMesh.worldUpperBound = upper;

            // make grid
            pNavMesh.xCount = (int)((upper.x - lower.x) / pNavMesh.gridSize.x) + 1;
            pNavMesh.yCount = (int)((upper.y - lower.y) / pNavMesh.gridSize.y) + 1;
            pNavMesh.zCount = (int)((upper.z - lower.z) / pNavMesh.gridSize.z) + 1;

            PNavColumn[,] columns = new PNavColumn[pNavMesh.xCount, pNavMesh.zCount];

            for (int x = 0; x < columns.GetLength(0); x++)
            {
                for (int z = 0; z < columns.GetLength(1); z++)
                {
                    PNavColumn column = new PNavColumn();
                    column.nodes = new PNavNode[pNavMesh.yCount];
                    column.type = ParallelNavColumnType.Empty;
                    column.surfaceNodeIndexes = new int[1];
                    column.surfaceNodeIndexes[0] = -1;
                    columns[x, z] = column;
                }
            }
            pNavMesh.columns = columns;

            using (new SProfiler($"CubeCast"))
            {
                int xStart1 = 0;
                int xEnd1 = columns.GetLength(0);
                int zStart1 = 0;
                int zEnd1 = columns.GetLength(1);
                int yStart1 = 0;
                int yEnd1 = pNavMesh.yCount;

                CubeCastInRange(pNavMesh, xStart1, xEnd1, zStart1, zEnd1, yStart1, yEnd1);

                /*
                int scale2 = 5;

                CheckVolume(xStart1, xEnd1, zStart1, zEnd1, yStart1, yEnd1, scale1, (xS, xE, zS, zE, yS, yE) =>
                {
                    CubeCastInRange(xS, xE, zS, zE, yS, yE);
                    CheckVolume(xS, xE, zS, zE, yS, yE, scale2, (xS2, xE2, zS2, zE2, yS2, yE2) =>
                    {
                        CubeCastInRange(xS2, xE2, zS2, zE2, yS2, yE2);
                    });

                });
                */
            }

            Parallel3D.CleanUp();
        }

        public static void CheckVolume(PNavMesh pNavMesh, int xStart, int xEnd, int zStart, int zEnd, int yStart, int yEnd, int scale, Action<int, int, int, int, int, int> callback)
        {
            PShapeOverlapResult3D result = new PShapeOverlapResult3D();

            Fix64Vec3 size = pNavMesh.gridSize * (Fix64)scale;
            Fix64Vec3 toCenter = Fix64.half * size;

            int scaledXEnd = xEnd / scale + 1;
            int scaledYEnd = yEnd / scale + 1;
            int scaledZEnd = zEnd / scale + 1;

            for (int x = xStart; x < scaledXEnd; x++)
            {
                for (int z = zStart; z < scaledZEnd; z++)
                {
                    for (int y = yStart; y < scaledZEnd; y++)
                    {
                        int scaledX = x * scale;
                        int scaledY = y * scale;
                        int scaledZ = z * scale;

                        Fix64Vec3 l;
                        Fix64Vec3 u;
                        pNavMesh.GetAABB(scaledX, scaledY, scaledZ, out l, out u);

                        Fix64Vec3 center = l + toCenter;
                        Fix64Quat rot = Fix64Quat.identity;

                        Parallel3D.OverlapCube(
                            center, rot,
                            size.x, size.y, size.z,
                            -1,
                            result);

                        if (result.count > 0)
                        {
                            callback(scaledX, scaledX + scale, scaledZ, scaledZ + scale, scaledY, scaledY + scale);
                        }
                    }
                }
            }
        }

        public static void CubeCastInRange(PNavMesh pNavMesh, int xStart, int xEnd, int zStart, int zEnd, int yStart, int yEnd)
        {
            PShapeOverlapResult3D result = new PShapeOverlapResult3D();

            if (xEnd > pNavMesh.xCount)
            {
                xEnd = pNavMesh.xCount;
            }

            if (zEnd > pNavMesh.zCount)
            {
                zEnd = pNavMesh.zCount;
            }

            if (yEnd > pNavMesh.yCount)
            {
                yEnd = pNavMesh.yCount;
            }

            Debug.Log("CubeCastInRange:" + " xStart=" + xStart + " xEnd=" + xEnd + " zStart=" + zStart + " zEnd=" + zEnd + " yStart=" + yStart + " yEnd=" + yEnd);

            for (int x = xStart; x < xEnd; x++)
            {
                for (int z = zStart; z < zEnd; z++)
                {
                    PNavColumn column = pNavMesh.columns[x, z];

                    int surfaceIndex = -1;

                    for (int y = yStart; y < yEnd; y++)
                    {
                        Fix64Vec3 l;
                        Fix64Vec3 u;
                        pNavMesh.GetAABB(x, y, z, out l, out u);
                        Fix64Vec3 center = Fix64.half * (l + u);
                        Fix64Quat rot = Fix64Quat.identity;
                        Parallel3D.OverlapCube(
                            center, rot,
                            pNavMesh.gridSize.x, pNavMesh.gridSize.y, pNavMesh.gridSize.z,
                            -1,
                            result
                            );

                        PNavNode node = new PNavNode();

                        node.lower = l;
                        node.upper = u;
                        node.objectCount = result.count;
                        node.islandIndex = -1;
                        node.point = new PNavPoint(x, z);

                        if (result.count > 0 && y > surfaceIndex)
                        {
                            surfaceIndex = y;
                        }

                        column.nodes[y] = node;
                    }

                    column.surfaceNodeIndexes[0] = surfaceIndex;

                    if (surfaceIndex >= 0)
                    {
                        column.type = ParallelNavColumnType.Walkable;
                    }
                    else
                    {
                        column.type = ParallelNavColumnType.Empty;
                    }
                }
            }
        }
    }

}