using System;
using System.Collections.Generic;
using Parallel;

namespace Parallel.Pathfinding
{
    public interface IAStarNode<T>
    {
        Fix64 F { get; set; }
        Fix64 G { get; set; }
        Fix64 H { get; set; }

        T UserObject { get; set; }
        IAStarNode<T> Parent { get; set; }

        bool Equals(T other);
        Fix64 CalculateG(T other);
        Fix64 CalculateH(T other);
    }

    public abstract class BaseAStarNode<T> : IIndexedObject, IAStarNode<T>
    {
        public Fix64 F { get; set; }
        public Fix64 G { get; set; }
        public Fix64 H { get; set; }

        public T UserObject { get; set; }
        public IAStarNode<T> Parent { get; set; }

        public BaseAStarNode(T userObject)
        {
            UserObject = userObject;
        }

        public abstract bool Equals(T other);
        public abstract Fix64 CalculateG(T other);
        public abstract Fix64 CalculateH(T other);

        public int Index { get; set; }
    }

    public abstract class BaseAStar<T, N> : IComparer<T> where T : IIndexedObject, IAStarNode<N>
    {
        HashSet<T> openSet;
        HashSet<T> clostSet;
        PriorityQueue<T> sortedQueue;
        T[] children;

        public abstract int ChildrenOfNode(T node, T[] children);
        public abstract void PrePathFinding();

        public BaseAStar()
        {
            openSet = new HashSet<T>();
            clostSet = new HashSet<T>();
            sortedQueue = new PriorityQueue<T>(this);
            children = new T[64];
        }

        public abstract int Compare(T x, T y);

        public T FindPath(T start, T end)
        {
            PrePathFinding();
            openSet.Clear();
            clostSet.Clear();
            sortedQueue.Clear();

            start.G = Fix64.zero;
            start.H = start.CalculateH(end.UserObject);
            start.F = start.G + start.H;

            openSet.Add(start);
            sortedQueue.Push(start);

            while (openSet.Count > 0)
            {
                T currentNode = sortedQueue.Pop();

                if (currentNode.Equals(end))
                {
                    return currentNode;
                }

                openSet.Remove(currentNode);
                clostSet.Add(currentNode);

                int childrenCount = ChildrenOfNode(currentNode, children);

                for (int i = 0; i < childrenCount; i++)//foreach(T child in chirldrenNodes)
                {
                    T child = children[i];
                    if (clostSet.Contains(child))
                    {
                        continue;
                    }

                    Fix64 tentativeG = currentNode.G + child.CalculateG(currentNode.UserObject);
                    bool shouldUpdate = false;

                    if (child.G == Fix64.zero)
                    {
                        shouldUpdate = true;
                    }
                    else
                    {
                        shouldUpdate = child.G > tentativeG;
                    }

                    if (shouldUpdate)
                    {
                        child.G = tentativeG;
                        child.H = child.CalculateH(end.UserObject);
                        child.F = child.G + child.H;
                        child.Parent = currentNode;
                    }

                    if (!openSet.Contains(child))
                    {
                        openSet.Add(child);
                        sortedQueue.Push(child);
                    }
                    else if (shouldUpdate)
                    {
                        sortedQueue.Update(child);
                    }
                }
            }

            return default(T);
        }
    }
}
