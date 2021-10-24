﻿using UnityEngine;
using UnityEditor;
using System.Collections;

namespace LYHSensor
{
    [CustomEditor(typeof(FOVCollider))]
    [CanEditMultipleObjects]
    public class FOVColliderEditor : Editor
    {
        SerializedProperty length;
        SerializedProperty baseSize;
        SerializedProperty fovAngle;
        SerializedProperty elevationAngle;
        SerializedProperty resolution;

        void OnEnable()
        {
            if (serializedObject == null) return;

            length = serializedObject.FindProperty("Length");
            baseSize = serializedObject.FindProperty("BaseSize");
            fovAngle = serializedObject.FindProperty("FOVAngle");
            elevationAngle = serializedObject.FindProperty("ElevationAngle");
            resolution = serializedObject.FindProperty("Resolution");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUILayout.PropertyField(length);
            EditorGUILayout.PropertyField(baseSize);
            EditorGUILayout.PropertyField(fovAngle);
            EditorGUILayout.PropertyField(elevationAngle);
            EditorGUILayout.PropertyField(resolution);

            serializedObject.ApplyModifiedProperties();
        }
    }
}