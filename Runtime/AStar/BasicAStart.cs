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
    }

    public abstract class BaseAStarNode<T> : IIndexedObject, IAStarNode<T>
    {
        public Fix64 F { get; set; }
        public Fix64 G { get; set; }
        public Fix64 H { get; set; }

        public T UserObject { get; set; }
        public BaseAStarNode<T> Parent { get; set; }

        public BaseAStarNode(T userObject)
        {
            UserObject = userObject;
        }

        public abstract bool Equals(BaseAStarNode<T> other);
        public abstract Fix64 CalculateG(BaseAStarNode<T> other);
        public abstract Fix64 CalculateH(BaseAStarNode<T> other);

        public int Index { get; set; }
    }

    public abstract class BaseAStar<T, N> : IComparer<T> where T : BaseAStarNode<N>, IIndexedObject
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
            openSet.Clear();
            clostSet.Clear();
            sortedQueue.Clear();

            start.G = Fix64.zero;
            start.H = start.CalculateH(end);
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

                    Fix64 tentativeG = currentNode.G + currentNode.CalculateG(child);
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
                        child.H = child.CalculateH(end);
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
