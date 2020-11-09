﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    [RequireComponent(typeof(ParallelTransform))]
    public class PNavMeshAgent : MonoBehaviour
    {
        public Fix64 movementSpeed = Fix64.FromDivision(1, 1);
        public Fix64 stopDistance = Fix64.FromDivision(1, 10);
        public Fix64 width = Fix64.FromDivision(1, 10);

        PNavMeshManager _navMeshManager;
        ParallelTransform _pTransform;

        Fix64Vec3 _destination;
        PNavMeshPath _path;

        public bool debug = false;
        public bool continuous = false;
        public List<Fix64Vec2> _waypoints = new List<Fix64Vec2>();

        void Start()
        {
            _navMeshManager = FindObjectOfType<PNavMeshManager>();
            _pTransform = GetComponent<ParallelTransform>();
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            if(_path == null)
            {
                return;
            }

            /*
            if (_nextCorner != Fix64Vec3.zero)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere((Vector3)_nextCorner, 0.1f);
            }
            */
            Gizmos.color = Color.red;

            if (_path.Status == ParallelNavMeshPathStatus.Valid)
            {
                if (_path.startIndex != -1)
                {
                    PNavPolygon previousPolygon = null;
                    for (int index = _path.startIndex; index < 128; index++)
                    {
                        int polygonIndex = _path.polygonIndexes[index];

                        PNavPolygon currentPolygon = _path.island.graph.polygons[polygonIndex];

                        if (previousPolygon != null)
                        {
                            Gizmos.DrawLine((Vector3)previousPolygon.Centroid3D, (Vector3)currentPolygon.Centroid3D);
                        }

                        previousPolygon = currentPolygon;
                    }

                    if (_minLeftVector != Fix64Vec2.zero)
                    {
                        Fix64Vec2 pos = _waypoints[_waypoints.Count - 1];
                        Vector3 pos3D = new Vector3((float)pos.x, 0, (float)pos.y);

                        Gizmos.color = Color.blue;
                        Vector3 left3D = new Vector3((float)_minLeft.x, 0, (float)_minLeft.y);
                        Gizmos.DrawLine(pos3D, left3D);
                        Gizmos.DrawWireSphere(left3D, 0.1f);

                        Gizmos.color = Color.magenta;
                        Vector3 right3D = new Vector3((float)_minRight.x, 0, (float)_minRight.y);

                        Gizmos.DrawLine(pos3D, right3D);
                        Gizmos.DrawWireSphere(right3D, 0.1f);
                    }
                }

                Gizmos.color = Color.white;
                for (int i = 0; i < _waypoints.Count; i++)
                {
                    Fix64Vec2 wp = _waypoints[i];
                    Vector3 wp3D = new Vector3((float)wp.x, 0, (float)wp.y);
                    Gizmos.DrawWireSphere(wp3D, 0.1f);

                    if (i == 0)
                    {
                        continue;
                    }
                    else
                    {
                        Fix64Vec2 prev = _waypoints[i - 1];
                        Vector3 prevWp3D = new Vector3((float)prev.x, 0, (float)prev.y);
                        Gizmos.DrawLine(prevWp3D, wp3D);
                    }
                }
            }
        }

        public void SetDestination(Fix64Vec3 destination)
        {
            _destination = destination;
            UpdatePath();
        }

        public Fix64Vec2 Position2D
        {
            get
            {
                return new Fix64Vec2(_pTransform.position.x, _pTransform.position.z);
            }
        }

        public void UpdatePath()
        {
            ResetPathValues();
            _waypoints.Clear();
            _currentWaypointTargetIndex = 0;
            _path = _navMeshManager.CalculatePath(_pTransform.position, _destination);

            if (_path.Status == ParallelNavMeshPathStatus.Valid)
            {
                _pathPolygonIndex = _path.startIndex;
                _waypoints.Add(Position2D);
                _finishedProcessing = false;

                if (_pathPolygonIndex == -1)
                {
                    Debug.Log("Already in destination polygon");
                    _waypoints.Add(_path.Destination2D);
                }
                else
                {
                    int polygonIndex = _path.polygonIndexes[_pathPolygonIndex];
                    _previousPolygon = _path.island.graph.polygons[polygonIndex];
                    _pathPolygonIndex++;

                    if(!debug)
                    {
                        int limit = 50;
                        while (!_finishedProcessing)
                        {
                            if (limit < 0)
                            {
                                Debug.LogError($"Unable to process path {limit}");
                                break;
                            }

                            ProcessNextPolygon();
                            limit--;
                        }
                    }
                }
            }
        }

        public void ResetPathValues()
        {
            _pathPolygonIndex = 0;
            _previousPolygon = null;
            _minLeft = Fix64Vec2.zero;
            _minRight = Fix64Vec2.zero;
            _minLeftVector = Fix64Vec2.zero;
            _minRightVector = Fix64Vec2.zero;
            _finishedProcessing = true;
            _leftPolygonIndex = 0;
            _rightPolygonIndex = 0;
            _leftPreviousPolygon = null;
            _rightPreviousPolygon = null;
        }

        int _pathPolygonIndex = 0;
        bool _finishedProcessing = true;
        PNavPolygon _previousPolygon = null;
        Fix64Vec2 _minLeft = Fix64Vec2.zero;
        Fix64Vec2 _minRight = Fix64Vec2.zero;
        Fix64Vec2 _minLeftVector = Fix64Vec2.zero;
        Fix64Vec2 _minRightVector = Fix64Vec2.zero;
        int _leftPolygonIndex = 0;
        PNavPolygon _leftPreviousPolygon = null;
        int _rightPolygonIndex = 0;
        PNavPolygon _rightPreviousPolygon = null;

        //http://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html
        void ProcessNextPolygon()
        {
            Debug.Log($"ProcessNextPolygon {_pathPolygonIndex}");
            Fix64Vec2 pos = _waypoints[_waypoints.Count - 1];

            if (_pathPolygonIndex == 128)
            {
                Debug.Log("LAST POLYGON");

                if(_minLeftVector == Fix64Vec2.zero)
                {
                    Debug.Log("GO to destination directly");
                    _waypoints.Add(_path.Destination2D);

                    ResetPathValues();
                    return;
                }

                Fix64Vec2 destinationVector = _path.Destination2D - pos;

                bool destinationLeftOfMinLeft = Fix64Vec2.Cross(_minLeftVector, destinationVector) > Fix64.zero;
                bool destinationRightOfMinRight = Fix64Vec2.Cross(_minRightVector, destinationVector) < Fix64.zero;

                bool lastRecalculate = false;
                bool useLeft = true;

                if(destinationLeftOfMinLeft && destinationRightOfMinRight)
                {
                    //when both are too be added
                    //use the one that is closer to pos
                    Fix64 leftDis = Fix64Vec2.Distance(_minLeft, pos);
                    Fix64 rightDis = Fix64Vec2.Distance(_minRight, pos);

                    if(leftDis > rightDis)
                    {
                        useLeft = false;
                    }
                }

                if (destinationLeftOfMinLeft && useLeft)
                {
                    _waypoints.Add(_minLeft);
                    pos = _minLeft;
                    _pathPolygonIndex = _leftPolygonIndex;
                    _previousPolygon = _leftPreviousPolygon;
                    lastRecalculate = true;
                }

                if (destinationRightOfMinRight && !lastRecalculate)
                {
                    _waypoints.Add(_minRight);
                    pos = _minRight;
                    _pathPolygonIndex = _rightPolygonIndex;
                    _previousPolygon = _rightPreviousPolygon;
                    lastRecalculate = true;
                }

                if(!lastRecalculate)
                {
                    Debug.Log("GO to destination directly");
                    _waypoints.Add(_path.Destination2D);

                    ResetPathValues();
                }
                else
                {
                    _minLeftVector = Fix64Vec2.zero;
                }

                return;
            }

            if (_finishedProcessing)
            {
                return;
            }

            int polygonIndex = _path.polygonIndexes[_pathPolygonIndex];

            PNavPolygon currentPolygon = _path.island.graph.polygons[polygonIndex];

            PNavEdge edge = _path.FindEdge(_previousPolygon, currentPolygon, width);

            if(edge == null)
            {
                Debug.Log("No Edge found");
                return;
            }

            _previousPolygon = currentPolygon;
            _pathPolygonIndex++;

            Fix64Vec2 ab = edge.pointB - edge.pointA;
            Fix64Vec2 ba = edge.pointA - edge.pointB;

            Fix64Vec2 pa = edge.pointA + ab.normalized * width;
            Fix64Vec2 pb = edge.pointB + ba.normalized * width;
            Fix64Vec2 vA = pa - pos;
            Fix64Vec2 vB = pb - pos;
            Fix64 c = Fix64Vec2.Cross(vA, vB);
            Fix64Vec2 left = pb;
            Fix64Vec2 right = pa;

            if (c > Fix64.zero)
            {
                Debug.Log($"POSITVE: pointA is on the right. pA={pa} pB={pb} pos={pos}");
            }
            else
            {
                Debug.Log($"NEGTIVE: pointA is on the left. pA={pa} pB={pb} pos={pos}");
                right = pb;
                left = pa;
            }

            if (_minLeftVector == Fix64Vec2.zero)
            {
                _minLeft = left;
                _minRight = right;
                _minLeftVector = left - pos;
                _minRightVector = right - pos;
                _leftPolygonIndex = _pathPolygonIndex;
                _rightPolygonIndex = _pathPolygonIndex;
                _leftPreviousPolygon = _previousPolygon;
                _rightPreviousPolygon = _previousPolygon;
                return;
            }

            Fix64Vec2 newLeftVector = left - pos;
            Fix64Vec2 newRightVector = right - pos;

            bool newLeftIsRightToOldRight = Fix64Vec2.Cross(_minRightVector, newLeftVector) < Fix64.zero;
            bool newRightIsLeftToOldLeft = Fix64Vec2.Cross(_minLeftVector, newRightVector) > Fix64.zero;

            bool recalculateNewVectors = false;

            if (newLeftIsRightToOldRight)
            {
                //move to old right and recalculate
                _waypoints.Add(_minRight);
                pos = _minRight;
                _pathPolygonIndex = _rightPolygonIndex;
                _previousPolygon = _rightPreviousPolygon;
                recalculateNewVectors = true;
            }

            if (newRightIsLeftToOldLeft && !recalculateNewVectors)
            {
                //move to old left and recalculate
                _waypoints.Add(_minLeft);
                pos = _minLeft;
                _pathPolygonIndex = _leftPolygonIndex;
                _previousPolygon = _leftPreviousPolygon;
                recalculateNewVectors = true;
            }

            if (recalculateNewVectors)
            {
                _minLeftVector = Fix64Vec2.zero;
                return;
            }
            else
            {
                bool newLeftIsOk = Fix64Vec2.Cross(_minLeftVector, newLeftVector) <= Fix64.zero;
                bool newRightIsOk = Fix64Vec2.Cross(_minRightVector, newRightVector) >= Fix64.zero;

                if(newLeftIsOk)
                {
                    _minLeft = left;
                    _minLeftVector = newLeftVector;
                    _leftPolygonIndex = _pathPolygonIndex;
                    _leftPreviousPolygon = _previousPolygon;
                }

                if(newRightIsOk)
                {
                    _minRight = right;
                    _minRightVector = newRightVector;
                    _rightPolygonIndex = _pathPolygonIndex;
                    _rightPreviousPolygon = _previousPolygon;
                }
            }
        }


        private void Update()
        {
            //Move();

            if (Input.GetKeyUp(KeyCode.N))
            {
                Debug.Log("=====Step=====");
                if (_path.Status == ParallelNavMeshPathStatus.Valid)
                {
                    ProcessNextPolygon();
                }
            }
        }

        int _currentWaypointTargetIndex = 0;
        public void Move(Fix64 deltaTime)
        {
            //using (new SProfiler($"move"))
            {
                if (_waypoints.Count > 0)
                {
                    Fix64Vec2 currentWaypoint = _waypoints[_currentWaypointTargetIndex];
                    Fix64Vec3 currentWaypoint3D = new Fix64Vec3(currentWaypoint.x, _pTransform.position.y, currentWaypoint.y);

                    //check if we have reached the waypoint
                    Fix64 distance = Fix64Vec3.Distance(_pTransform.position, currentWaypoint3D);

                    if (distance > stopDistance)
                    {
                        Fix64Vec3 direction = currentWaypoint3D - _pTransform.position;
                        Fix64 stepDistance = movementSpeed * Fix64.FromDivision(2, 100);
                        stepDistance = Fix64Math.Min(distance, stepDistance);
                        _pTransform.position += direction.normalized * stepDistance;
                    }
                    else
                    {
                        _currentWaypointTargetIndex++;

                        if (_currentWaypointTargetIndex == _waypoints.Count)
                        {
                            //reached last waypoint
                            // DONE
                            _waypoints.Clear();
                            _currentWaypointTargetIndex = 0;
                        }
                    }
                }
            }
        }
        private void FixedUpdate()
        {
            if(!debug)
            {
               Move(Fix64.FromDivision(2, 100));
            }
        }

        /*
        private void Update()
        {
            if (_path.Status == ParallelNavMeshPathStatus.Valid)
            {
                Fix64Vec2 pos2D = new Fix64Vec2(_pTransform.position.x, _pTransform.position.z);
                Fix64Vec2 cornerPos2D = new Fix64Vec2(_nextCorner.x, _nextCorner.z);
                distanceToNextCorner = Fix64Vec2.Distance(pos2D, cornerPos2D);

                if (_nextCorner == Fix64Vec3.zero || distanceToNextCorner < stopDistance)
                {
                    //get next corner
                    bool foundNextCorner = _path.NextCorner(_pTransform.position, width, ref _nextCorner, ref _nextPolygonIndex);
                    _nextCorner = new Fix64Vec3(_nextCorner.x, _pTransform.position.y, _nextCorner.z);

                    if (!foundNextCorner)
                    {
                        Debug.Log("Failed to find the next corner, try calculate a new path");
                        //UpdatePath();
                    }
                }
                else
                {
                    //move towards the corner
                    Fix64Vec3 direction = _nextCorner - _pTransform.position;
                    _pTransform.position = transform.position + movementSpeed * direction.normalized * Time.deltaTime;
                }
            }
        }
        */
    }

}
