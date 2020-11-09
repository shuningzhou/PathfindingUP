using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using Parallel;

namespace Parallel.Pathfinding
{
    public class ParallelNavigationWindow : EditorWindow
    {
        PNavMesh pNavMesh;
        Material defaultDrawingMaterial;
        PShapeOverlapResult3D result = new PShapeOverlapResult3D();
        SerializedObject so;

        bool drawAAABB = false;
        bool drawPolygonGraph = false;
        bool drawEdgeLoop = false;
        bool drawWalkAble = false;
        bool drawEdge = false;
        bool drawEdgeGap = false;
        bool drawPolygon = true;
        bool drawCorner = false;

        bool debugDraw = false;

        [MenuItem("Parallel/Navigation")]
        public static void ShowWindow()
        {
            EditorWindow.GetWindow(typeof(ParallelNavigationWindow));
        }

        void OnEnable()
        {
            if (EditorPrefs.HasKey("ObjectPath"))
            {
                string objectPath = EditorPrefs.GetString("ObjectPath");
                pNavMesh = AssetDatabase.LoadAssetAtPath(objectPath, typeof(PNavMesh)) as PNavMesh;
                if (pNavMesh)
                {
                    so = new SerializedObject(pNavMesh);
                }
            }

            SceneView.duringSceneGui += this.OnSceneGUI;
        }

        void OnDisable()
        {
            SceneView.duringSceneGui -= this.OnSceneGUI;
        }

        void OnSceneGUI(SceneView sceneView)
        {
            if (Event.current.type != EventType.Repaint)
            {
                return;
            }

            if(!debugDraw)
            {
                return;
            }

            if (pNavMesh && !pNavMesh.saved)
            {
                if (drawAAABB)
                {
                    PNavMeshDebugDraw.DrawAABB(pNavMesh);
                }

                if (drawPolygonGraph)
                {
                    PNavMeshDebugDraw.DrawPolygonGraph(pNavMesh);
                }

                if (drawEdgeLoop)
                {
                    PNavMeshDebugDraw.DrawEdgeLoop(pNavMesh);
                }

                if (drawEdge)
                {
                    PNavMeshDebugDraw.DrawEdge(pNavMesh);
                }

                if (drawWalkAble)
                {
                    PNavMeshDebugDraw.DrawWalkable(pNavMesh);
                }

                if (drawEdgeGap)
                {
                    PNavMeshDebugDraw.DrawEdgeGap(pNavMesh);
                }

                if (drawPolygon)
                {
                    PNavMeshDebugDraw.DrawPolygon(pNavMesh);
                }

                if (drawCorner)
                {
                    PNavMeshDebugDraw.DrawCorner(pNavMesh);
                }
            }
        }

        void OnGUI()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Navigation", EditorStyles.boldLabel);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();

            if (GUILayout.Button("Select"))
            {
                ShowSelectWndow();
            }

            GUILayout.EndHorizontal();

            if (pNavMesh)
            {
                GUILayout.BeginHorizontal();
                pNavMesh.verticalDrop = EditorGUILayout.IntField("Vertical Drop", pNavMesh.verticalDrop);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                pNavMesh.edgeGap = EditorGUILayout.IntField("Edge Gap", pNavMesh.edgeGap);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                EditorGUILayout.PropertyField(so.FindProperty("gridSize"));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                EditorGUILayout.PropertyField(so.FindProperty("worldOrigin"));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                EditorGUILayout.PropertyField(so.FindProperty("worldSize"));
                GUILayout.EndHorizontal();

                so.ApplyModifiedProperties();

                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Update"))
                {
                    UpdateNavMesh();
                }

                if (GUILayout.Button("Save"))
                {
                    SaveNavMesh();
                }

                GUILayout.EndHorizontal();

                if(!pNavMesh.saved)
                {
                    //debug
                    GUILayout.BeginHorizontal();
                    debugDraw = EditorGUILayout.Toggle("Debug Draw", debugDraw);
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    drawEdgeLoop = EditorGUILayout.Toggle("Draw Edge Loop", drawEdgeLoop);
                    drawWalkAble = EditorGUILayout.Toggle("Draw Walkable", drawWalkAble);
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    drawEdge = EditorGUILayout.Toggle("Draw Edge", drawEdge);
                    drawEdgeGap = EditorGUILayout.Toggle("Draw Edge Gap", drawEdgeGap);
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    drawPolygon = EditorGUILayout.Toggle("Draw Polygon", drawPolygon);
                    drawCorner = EditorGUILayout.Toggle("Draw Corner", drawCorner);
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    drawAAABB = EditorGUILayout.Toggle("Draw AABB", drawAAABB);
                    drawPolygonGraph = EditorGUILayout.Toggle("Draw Polygon Graph", drawPolygonGraph);
                    GUILayout.EndHorizontal();
                }
            }
        }

        void ShowSelectWndow()
        {
            string[] filters = { "Parallel NavMesh", "asset" };
            string absPath = EditorUtility.OpenFilePanelWithFilters("Select Parallel NavMesh ScriptableObject", "Assets", filters);
            //string absPath = EditorUtility.OpenFilePanel("Select Parallel NavMesh ScriptableObject", "", "");

            if (absPath.StartsWith(Application.dataPath))
            {
                string relPath = absPath.Substring(Application.dataPath.Length - "Assets".Length);
                pNavMesh = AssetDatabase.LoadAssetAtPath(relPath, typeof(PNavMesh)) as PNavMesh;

                if (pNavMesh)
                {
                    EditorPrefs.SetString("ObjectPath", relPath);
                    so = new SerializedObject(pNavMesh);
                }
            }
        }

        void UpdateNavMesh()
        {
            pNavMesh.Reset();

            NavMeshGenerator navMeshGenerator = new NavMeshGenerator(pNavMesh);
            navMeshGenerator.Generate();

            Debug.Log("NavMesh updated");
        }

        void SaveNavMesh()
        {
            pNavMesh.Save();
            EditorUtility.SetDirty(pNavMesh);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("NavMesh saved");
        }
        //    void CreateCubeMesh()
        //    {
        //        Vector3 lower = (Vector3)pNavMesh.worldLowerBound;
        //        Vector3 upper = (Vector3)pNavMesh.worldUpperBound;

        //        Vector3 center = 0.5f * (lower + upper);
        //        float length = upper.x - lower.x;
        //        float width = upper.y - lower.y;
        //        float height = upper.z - lower.z;

        //        //https://gist.github.com/prucha/866b9535d525adc984c4fe883e73a6c7

        //        Vector3[] c = new Vector3[8];

        //        c[0] = new Vector3(-length * .5f, -width * .5f, height * .5f);
        //        c[1] = new Vector3(length * .5f, -width * .5f, height * .5f);
        //        c[2] = new Vector3(length * .5f, -width * .5f, -height * .5f);
        //        c[3] = new Vector3(-length * .5f, -width * .5f, -height * .5f);

        //        c[4] = new Vector3(-length * .5f, width * .5f, height * .5f);
        //        c[5] = new Vector3(length * .5f, width * .5f, height * .5f);
        //        c[6] = new Vector3(length * .5f, width * .5f, -height * .5f);
        //        c[7] = new Vector3(-length * .5f, width * .5f, -height * .5f);

        //        Vector3[] vertices = new Vector3[]
        //{
        //        c[0], c[1], c[2], c[3], // Bottom
        //     c[7], c[4], c[0], c[3], // Left
        //     c[4], c[5], c[1], c[0], // Front
        //     c[6], c[7], c[3], c[2], // Back
        //     c[5], c[6], c[2], c[1], // Right
        //     c[7], c[6], c[5], c[4]  // Top
        //};

        //        Vector3 up = Vector3.up;
        //        Vector3 down = Vector3.down;
        //        Vector3 forward = Vector3.forward;
        //        Vector3 back = Vector3.back;
        //        Vector3 left = Vector3.left;
        //        Vector3 right = Vector3.right;

        //        Vector3[] normals = new Vector3[]
        //        {
        //        down, down, down, down,             // Bottom
        //     left, left, left, left,             // Left
        //     forward, forward, forward, forward,	// Front
        //     back, back, back, back,             // Back
        //     right, right, right, right,         // Right
        //     up, up, up, up                      // Top
        //        };

        //        Vector2 uv00 = new Vector2(0f, 0f);
        //        Vector2 uv10 = new Vector2(1f, 0f);
        //        Vector2 uv01 = new Vector2(0f, 1f);
        //        Vector2 uv11 = new Vector2(1f, 1f);

        //        Vector2[] uvs = new Vector2[]
        //        {
        //        uv11, uv01, uv00, uv10, // Bottom
        //     uv11, uv01, uv00, uv10, // Left
        //     uv11, uv01, uv00, uv10, // Front
        //     uv11, uv01, uv00, uv10, // Back	        
        //     uv11, uv01, uv00, uv10, // Right 
        //     uv11, uv01, uv00, uv10  // Top
        //        };

        //        int[] triangles = new int[]
        //        {
        //        3, 1, 0,        3, 2, 1,        // Bottom	
        //     7, 5, 4,        7, 6, 5,        // Left
        //     11, 9, 8,       11, 10, 9,      // Front
        //     15, 13, 12,     15, 14, 13,     // Back
        //     19, 17, 16,     19, 18, 17,	    // Right
        //     23, 21, 20,     23, 22, 21,     // Top
        //        };

        //        Mesh mesh = new Mesh();
        //        mesh.vertices = vertices;
        //        mesh.triangles = triangles;
        //        mesh.normals = normals;
        //        mesh.uv = uvs;

        //        pNavMesh.mesh = mesh;

        //        Fix64Vec3 flower = pNavMesh.worldLowerBound;
        //        Fix64Vec3 fupper = pNavMesh.worldUpperBound;

        //        Fix64Vec3 fcenter = Fix64.half * (flower + fupper);

        //        pNavMesh.meshCenter = fcenter;

        //    }

        Material defaultMaterial()
        {
            Material standardShaderMaterial = new Material(Shader.Find("Standard"));
            standardShaderMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            standardShaderMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            standardShaderMaterial.SetInt("_ZWrite", 0);
            standardShaderMaterial.DisableKeyword("_ALPHATEST_ON");
            standardShaderMaterial.EnableKeyword("_ALPHABLEND_ON");
            standardShaderMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            standardShaderMaterial.renderQueue = 3000;
            standardShaderMaterial.color = new Color(0, 1, 0, 0.01f);
            return standardShaderMaterial;
        }
    }
}