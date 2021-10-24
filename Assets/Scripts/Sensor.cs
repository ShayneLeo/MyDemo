using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace LYHSensor
{
    /*
     *  Sensors can run in two detection modes
     *  - Colliders: The sensor detects the GameObject attached to any collider it intersects.
     *  - RigidBodies: The sensor detects the GameObject owning the attached RigidBody of any collider it intersects.
     */
    public enum SensorMode { Colliders, RigidBodies }

    public class TagSelectorAttribute : PropertyAttribute { }

    /*
     *  Base class implemented by all sensor types with common functions for querying and filtering
     *  the sensors list of detected objects.
     */
    public abstract class Sensor : MonoBehaviour
    {
        [Tooltip("Any GameObject in this list will not be detected by this sensor, however it may still block line of sight.")]
        public List<GameObject> IgnoreList;

        [Tooltip("When set to true the sensor will only detect objects whose tags are in the 'TagFilter' array.")]
        public bool EnableTagFilter;

        [Tooltip("Array of tags that will be detected by the sensor.")]
        [TagSelector]
        public string[] AllowedTags;

        // Should return a list of all detected GameObjects, not necessarily in any order.
        public abstract List<GameObject> DetectedObjects { get; }

        // Should return a list of all detected GameObjects in order of distance from the sensor.
        public abstract List<GameObject> DetectedObjectsOrderedByDistance { get; }

        [System.Serializable]
        public class SensorEventHandler : UnityEvent<Sensor> { }

        [System.Serializable]
        public class SensorDetectionEventHandler : UnityEvent<GameObject, Sensor> { }

        // Event is called for each GameObject at the time it is added to the sensors DetectedObjects list
        [SerializeField]
        public SensorDetectionEventHandler OnDetected;

        // Event is called for each GameObject at the time it is removed to the sensors DetectedObjects list
        [SerializeField]
        public SensorDetectionEventHandler OnLostDetection;

        protected virtual void Awake()
        {
            if (IgnoreList == null)
            {
                IgnoreList = new List<GameObject>();
            }

            if (OnDetected == null) 
            {
                OnDetected = new SensorDetectionEventHandler();
            }

            if (OnLostDetection == null)
            {
                OnLostDetection = new SensorDetectionEventHandler();
            }
        }

        // Returns true when the passed GameObject is currently detected by the sensor, false otherwise.
        public virtual bool IsDetected(GameObject go)
        {
            var detectedEnumerator = DetectedObjects.GetEnumerator();
            while (detectedEnumerator.MoveNext())
            {
                if (detectedEnumerator.Current == go) { return true; }
            }
            return false;
        }

        public virtual float GetVisibility(GameObject go)
        {
            return IsDetected(go) ? 1f : 0f;
        }

        public abstract void Pulse();

        public List<GameObject> GetDetected()
        {
            return new List<GameObject>(DetectedObjectsOrderedByDistance);
        }
        public GameObject GetNearestToPoint(Vector3 p)
        {
            return nearestToPoint(DetectedObjects, p);
        }

        public GameObject GetNearest()
        {
            return nearestToPoint(DetectedObjects, transform.position);
        }

        protected bool shouldIgnore(GameObject go)
        {
            if (EnableTagFilter)
            {
                var tagFound = false;
                for (int i = 0; i < AllowedTags.Length; i++)
                {
                    if (AllowedTags[i] != "" && go != null && go.CompareTag(AllowedTags[i]))
                    {
                        tagFound = true;
                        break;
                    }
                }
                if (!tagFound) return true;
            }
            for (int i = 0; i < IgnoreList.Count; i++)
            {
                if (IgnoreList[i] == go) return true;
            }
            return false;
        }

        private GameObject nearestToPoint(List<GameObject> gos, Vector3 point)
        {
            GameObject nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = go;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private T nearestToPointWithComponent<T>(List<GameObject> gos, Vector3 point) where T : Component
        {
            T nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                var c = go.GetComponent<T>();
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private Component nearestToPointWithComponent(List<GameObject> gos, Vector3 point, Type t)
        {
            Component nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                var c = go.GetComponent(t);
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private GameObject nearestToPointWithName(List<GameObject> gos, Vector3 point, string name)
        {
            GameObject nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (go.name != name) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = go;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private T nearestToPointWithNameAndComponent<T>(List<GameObject> gos, Vector3 point, string name) where T : Component
        {
            T nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (go.name != name) { continue; }
                var c = go.GetComponent<T>();
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private Component nearestToPointWithNameAndComponent(List<GameObject> gos, Vector3 point, string name, Type t)
        {
            Component nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (go.name != name) { continue; }
                var c = go.GetComponent(t);
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private GameObject nearestToPointWithTag(List<GameObject> gos, Vector3 point, string tag)
        {
            GameObject nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (!go.CompareTag(tag)) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = go;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private T nearestToPointWithTagAndComponent<T>(List<GameObject> gos, Vector3 point, string tag) where T : Component
        {
            T nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (!go.CompareTag(tag)) { continue; }
                var c = go.GetComponent<T>();
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private Component nearestToPointWithTagAndComponent(List<GameObject> gos, Vector3 point, string tag, Type t)
        {
            Component nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (!go.CompareTag(tag)) { continue; }
                var c = go.GetComponent(t);
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private GameObject nearestToPointWithNameAndTag(List<GameObject> gos, Vector3 point, string name, string tag)
        {
            GameObject nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (go.name != name || !go.CompareTag(tag)) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = go;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private T nearestToPointWithNameAndTagAndComponent<T>(List<GameObject> gos, Vector3 point, string name, string tag) where T : Component
        {
            T nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (go.name != name || !go.CompareTag(tag)) { continue; }
                var c = go.GetComponent<T>();
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }

        private Component nearestToPointWithNameAndTagAndComponent(List<GameObject> gos, Vector3 point, string name, string tag, Type t)
        {
            Component nearest = null;
            var nearestDistance = 0f;
            var gosEnumerator = gos.GetEnumerator();
            while (gosEnumerator.MoveNext())
            {
                var go = gosEnumerator.Current;
                if (go.name != name || !go.CompareTag(tag)) { continue; }
                var c = go.GetComponent(t);
                if (c == null) { continue; }
                var d = Vector3.SqrMagnitude(go.transform.position - point);
                if (nearest == null || d < nearestDistance)
                {
                    nearest = c;
                    nearestDistance = d;
                }
            }
            return nearest;
        }
    }

    public class DistanceFromPointComparer : IComparer<GameObject>
    {
        public Vector3 Point;

        public int Compare(GameObject x, GameObject y)
        {
            var d1 = Vector3.SqrMagnitude(x.transform.position - Point);
            var d2 = Vector3.SqrMagnitude(y.transform.position - Point);
            if (d1 < d2) { return -1; }
            else if (d1 > d2) { return 1; }
            else { return 0; }
        }
    }
}