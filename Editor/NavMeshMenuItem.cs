using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace Parallel.Pathfinding
{
    public class NavMeshMenuItem
    {
        [MenuItem("Parallel/Create New NavMesh")]
        static void CreateNewNavMesh()
        {
            PNavMesh newNavMesh = ScriptableObject.CreateInstance<PNavMesh>();

            AssetDatabase.CreateAsset(newNavMesh, "Assets/NewParallelNavMesh.asset");
            AssetDatabase.SaveAssets();

            EditorUtility.FocusProjectWindow();
            Selection.activeObject = newNavMesh;
        }
    }
}
