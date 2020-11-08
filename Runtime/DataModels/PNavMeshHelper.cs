
using System.Collections.Generic;
using System.Runtime.Serialization;

namespace Parallel.Pathfinding
{
    public class PNavMeshHelper
    {
        static PNavPoint LEFT = new PNavPoint(-1, 0);
        static PNavPoint TOP = new PNavPoint(0, 1);
        static PNavPoint RIGHT = new PNavPoint(1, 0);
        static PNavPoint BOTTOM = new PNavPoint(0, -1);

        static PNavPoint TOPLEFT = new PNavPoint(-1, 1);
        static PNavPoint TOPRIGHT = new PNavPoint(1, 1);
        static PNavPoint BOTTOMLEFT = new PNavPoint(-1, -1);
        static PNavPoint BOTTOMRIGHT = new PNavPoint(1, -1);

        public static void ApplyEdgeGap(PNavColumn[,] columns, PNavPoint pointMax, PNavIsland island, int edgeGap)
        {
            foreach (PNavNode node in island.nodes)
            {
                if (node.IsInner)
                {

                    // remove narrow path

                    // check top and bot
                    bool top2Valid = isInnerNode(node.point, pointMax, TOP, 2, columns);
                    bool top1Valid = isInnerNode(node.point, pointMax, TOP, 1, columns);
                    bool bot1Valid = isInnerNode(node.point, pointMax, BOTTOM, 1, columns);
                    bool bot2Valid = isInnerNode(node.point, pointMax, BOTTOM, 2, columns);

                    if (top2Valid && top1Valid || top1Valid && bot1Valid || bot1Valid && bot2Valid)
                    {
                        // valid
                    }
                    else
                    {
                        node.walkable = false;
                    }

                    // check left and right
                    bool left2Valid = isInnerNode(node.point, pointMax, LEFT, 2, columns);
                    bool left1Valid = isInnerNode(node.point, pointMax, LEFT, 1, columns);
                    bool right1Valid = isInnerNode(node.point, pointMax, RIGHT, 1, columns);
                    bool right2Valid = isInnerNode(node.point, pointMax, RIGHT, 2, columns);

                    if (left2Valid && left1Valid || left1Valid && right1Valid || right1Valid && right2Valid)
                    {
                        // valid
                    }
                    else
                    {
                        node.walkable = false;
                    }

                    // diagonals
                    //bool topLeft2Valid = isInnerNode(node.point, pointMax, TOPLEFT, 2, columns);
                    //bool topLeft1Valid = isInnerNode(node.point, pointMax, TOPLEFT, 1, columns);
                    //bool botRight1Valid = isInnerNode(node.point, pointMax, BOTTOMRIGHT, 1, columns);
                    //bool botRight2Valid = isInnerNode(node.point, pointMax, BOTTOMRIGHT, 2, columns);

                    //if (topLeft2Valid && topLeft1Valid || topLeft1Valid && botRight1Valid || botRight1Valid && botRight2Valid)
                    //{
                    //    // valid
                    //}
                    //else
                    //{
                    //    node.walkable = false;
                    //}

                    //bool topRight2Valid = isInnerNode(node.point, pointMax, TOPRIGHT, 2, columns);
                    //bool topRight1Valid = isInnerNode(node.point, pointMax, TOPRIGHT, 1, columns);
                    //bool botLeft1Valid = isInnerNode(node.point, pointMax, BOTTOMLEFT, 1, columns);
                    //bool botLeft2Valid = isInnerNode(node.point, pointMax, BOTTOMLEFT, 2, columns);

                    //if (topRight2Valid && topRight1Valid || topRight1Valid && botLeft1Valid || botLeft1Valid && botLeft2Valid)
                    //{
                    //    // valid
                    //}
                    //else
                    //{
                    //    node.walkable = false;
                    //}

                    continue;
                }
                else
                {
                    node.walkable = false;

                    // remove edgeGap
                    //left
                    for (int i = 0; i < edgeGap; i++)
                    {
                        //left
                        {
                            PNavPoint pOut;
                            bool valid = GetLeftPoint(node.point, pointMax, i, out pOut);
                            if (valid)
                            {
                                PNavNode n = columns[pOut.x, pOut.z].SurfaceNode();
                                if (n != null)
                                {
                                    n.walkable = false;
                                }
                            }
                        }

                        //right
                        {
                            PNavPoint pOut;
                            bool valid = GetRightPoint(node.point, pointMax, i, out pOut);
                            if (valid)
                            {
                                PNavNode n = columns[pOut.x, pOut.z].SurfaceNode();
                                if (n != null)
                                {
                                    n.walkable = false;
                                }
                            }
                        }

                        //top
                        {
                            PNavPoint pOut;
                            bool valid = GetTopPoint(node.point, pointMax, i, out pOut);
                            if (valid)
                            {
                                PNavNode n = columns[pOut.x, pOut.z].SurfaceNode();
                                if (n != null)
                                {
                                    n.walkable = false;
                                }
                            }
                        }

                        //bottom
                        {
                            PNavPoint pOut;
                            bool valid = GetBottomPoint(node.point, pointMax, i, out pOut);
                            if (valid)
                            {
                                PNavNode n = columns[pOut.x, pOut.z].SurfaceNode();
                                if (n != null)
                                {
                                    n.walkable = false;
                                }
                            }
                        }
                    }
                }
            }
        }

        public static bool AddLeft(PNavColumn[,] columns, PNavPoint point, out PNavPoint pOut, PNavPoint pointMax, int verticalDrop, int edgeGap, int islandIndex, PNavIsland island, Queue<PNavPoint> queue)
        {
            PNavColumn column = columns[point.x, point.z];
            PNavNode node = column.SurfaceNode();

            bool valid = GetLeftPoint(point, pointMax, 1, out pOut);
            bool connected = false;
            bool alreadyAdded = false;

            if (valid)
            {
                PNavColumn c = columns[pOut.x, pOut.z];
                PNavNode sn = c.SurfaceNode();

                //already checked
                if (sn != null && sn.islandIndex >= 0)
                {
                    alreadyAdded = true;
                }

                connected = column.IsConnected(c, verticalDrop);

                if (connected && !alreadyAdded)
                {

                    sn.islandIndex = islandIndex;
                    island.nodes.Add(sn);

                    //add top to queue
                    PNavPoint pTopOut;

                    bool addedTop = AddTop(columns, pOut, out pTopOut, pointMax, verticalDrop, edgeGap, islandIndex, island);

                    if (addedTop)
                    {
                        queue.Enqueue(pTopOut);
                    }

                    //add bottom to queue
                    PNavPoint pBottomOut;
                    bool addedBottom = AddBottom(columns, pOut, out pBottomOut, pointMax, verticalDrop, edgeGap, islandIndex, island);

                    if (addedBottom)
                    {
                        queue.Enqueue(pBottomOut);
                    }
                }
            }

            if (!connected)
            {
                node.type = node.type | (int)ParallelNavIslandNodeType.LeftEdge;
            }

            return connected && !alreadyAdded;
        }

        public static bool AddRight(PNavColumn[,] columns, PNavPoint point, out PNavPoint pOut, PNavPoint pointMax, int verticalDrop, int edgeGap, int islandIndex, PNavIsland island, Queue<PNavPoint> queue)
        {
            PNavColumn column = columns[point.x, point.z];
            PNavNode node = column.SurfaceNode();

            bool valid = GetRightPoint(point, pointMax, 1, out pOut);
            bool connected = false;
            bool alreadyAdded = false;

            if (valid)
            {
                PNavColumn c = columns[pOut.x, pOut.z];
                PNavNode sn = c.SurfaceNode();

                //already checked
                if (sn != null && sn.islandIndex >= 0)
                {
                    alreadyAdded = true;
                }

                connected = column.IsConnected(c, verticalDrop);

                if (connected && !alreadyAdded)
                {
                    sn.islandIndex = islandIndex;
                    island.nodes.Add(sn);

                    //add top to queue
                    PNavPoint pTopOut;

                    bool addedTop = AddTop(columns, pOut, out pTopOut, pointMax, verticalDrop, edgeGap, islandIndex, island);

                    if (addedTop)
                    {
                        queue.Enqueue(pTopOut);
                    }

                    //add bottom to queue
                    PNavPoint pBottomOut;
                    bool addedBottom = AddBottom(columns, pOut, out pBottomOut, pointMax, verticalDrop, edgeGap, islandIndex, island);

                    if (addedBottom)
                    {
                        queue.Enqueue(pBottomOut);
                    }
                }
            }

            if (!connected)
            {
                node.type = node.type | (int)ParallelNavIslandNodeType.RightEdge;
            }

            return connected && !alreadyAdded;
        }

        public static bool AddTop(PNavColumn[,] columns, PNavPoint point, out PNavPoint pOut, PNavPoint pointMax, int verticalDrop, int edgeGap, int islandIndex, PNavIsland island)
        {
            PNavColumn column = columns[point.x, point.z];
            PNavNode node = column.SurfaceNode();

            bool valid = GetTopPoint(point, pointMax, edgeGap, out pOut);
            bool connected = false;
            bool alreadyAdded = false;

            if (valid)
            {
                PNavColumn c = columns[pOut.x, pOut.z];
                PNavNode sn = c.SurfaceNode();

                //already checked
                if (sn != null && sn.islandIndex >= 0)
                {
                    alreadyAdded = true;
                }

                connected = column.IsConnected(c, verticalDrop);

                if (connected && !alreadyAdded)
                {
                    sn.islandIndex = islandIndex;
                    island.nodes.Add(sn);
                }
            }

            if (!connected)
            {
                node.type = node.type | (int)ParallelNavIslandNodeType.TopEdge;
            }

            return connected && !alreadyAdded;
        }


        public static bool AddBottom(PNavColumn[,] columns, PNavPoint point, out PNavPoint pOut, PNavPoint pointMax, int verticalDrop, int edgeGap, int islandIndex, PNavIsland island)
        {
            PNavColumn column = columns[point.x, point.z];
            PNavNode node = column.SurfaceNode();

            bool valid = GetBottomPoint(point, pointMax, edgeGap, out pOut);
            bool connected = false;
            bool alreadyAdded = false;

            if (valid)
            {
                PNavColumn c = columns[pOut.x, pOut.z];
                PNavNode sn = c.SurfaceNode();

                //already checked
                if (sn != null && sn.islandIndex >= 0)
                {
                    alreadyAdded = true;
                }

                connected = column.IsConnected(c, verticalDrop);

                if (connected && !alreadyAdded)
                {
                    sn.islandIndex = islandIndex;
                    island.nodes.Add(sn);
                }
            }

            if (!connected)
            {
                node.type = node.type | (int)ParallelNavIslandNodeType.BottomEdge;
            }

            return connected && !alreadyAdded;
        }

        public static bool GetPoint(PNavPoint point, PNavPoint pointMax, PNavPoint nodes, out PNavPoint pointOut)
        {
            pointOut = new PNavPoint(point.x + nodes.x, point.z + nodes.z);

            if (pointOut.x > pointMax.x || pointOut.x < 0 || pointOut.z > pointMax.z || pointOut.z < 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public static bool GetRightPoint(PNavPoint point, PNavPoint pointMax, int nodes, out PNavPoint pointOut)
        {
            pointOut = new PNavPoint(point.x + nodes, point.z);

            if (pointOut.x > pointMax.x)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public static bool GetLeftPoint(PNavPoint point, PNavPoint pointMax, int nodes, out PNavPoint pointOut)
        {
            pointOut = new PNavPoint(point.x - nodes, point.z);

            if (pointOut.x < 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public static bool GetTopPoint(PNavPoint point, PNavPoint pointMax, int nodes, out PNavPoint pointOut)
        {
            pointOut = new PNavPoint(point.x, point.z + nodes);

            if (pointOut.z > pointMax.z)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public static bool GetBottomPoint(PNavPoint point, PNavPoint pointMax, int nodes, out PNavPoint pointOut)
        {
            pointOut = new PNavPoint(point.x, point.z - nodes);

            if (pointOut.z < 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        static bool isInnerNode(PNavPoint point, PNavPoint max, PNavPoint delta, int nodes, PNavColumn[,] columns)
        {
            PNavPoint pOut;
            PNavPoint newDelta = new PNavPoint(delta.x * nodes, delta.z * nodes);
            bool valid = PNavMeshHelper.GetPoint(point, max, newDelta, out pOut);

            if (valid)
            {
                PNavNode n = columns[pOut.x, pOut.z].SurfaceNode();

                valid = (n != null && n.IsInner);
            }

            return valid;
        }

        public static bool GetLeft(int x, int z, int maxX, int maxZ, out int xOut, out int zOout)
        {
            xOut = x - 1;
            zOout = z;

            if (xOut < 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public static bool GetRight(int x, int z, int maxX, int maxZ, out int xOut, out int zOout)
        {
            xOut = x + 1;
            zOout = z;

            if (xOut > maxX)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public static bool GetFront(int x, int z, int maxX, int maxZ, out int xOut, out int zOout)
        {
            xOut = x;
            zOout = z + 1;

            if (zOout < maxZ)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public static bool GetBack(int x, int z, int maxX, int maxZ, out int xOut, out int zOout)
        {
            xOut = x;
            zOout = z - 1;

            if (zOout < 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

}