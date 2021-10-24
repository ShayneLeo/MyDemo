using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace LYHSensor
{
    [ExecuteInEditMode]
    public abstract class BaseVolumeSensor : Sensor
    {
        [Tooltip("在碰撞器模式下，传感器检测附加到碰撞器上的游戏对象。 在RigidBody模式中，它检测附加到碰撞器的RigidBody游戏对象。  ")]
        public SensorMode DetectionMode;

        [Tooltip("只有通过视线测试的GameObject才会被检测到。")]
        public bool RequiresLineOfSight;

        [Tooltip("视线射线将在这个图层蒙版上测试")]
        public LayerMask BlocksLineOfSight;

        [Tooltip("如果为真，则只对带有LOSTargets组件的对象执行视线测试，如果为假，则传感器将自动为没有此组件的对象生成测试点。  ")]
        public bool TestLOSTargetsOnly;

        [Range(1, 20), Tooltip("Sensor将在没有LOSTargets组件的对象上生成的测试点数量。")]
        public int NumberOfRays = 1;

        public float MinimumVisibility = 0.5f;

        public HashSet<GameObject> ShowRayCastDebug;

        public override List<GameObject> DetectedObjects 
        {
            get
            {
                detectedObjects.Clear();

                if (RequiresLineOfSight)
                {
                    var objectVisibilityEnumerator = objectVisibility.GetEnumerator();
                    while (objectVisibilityEnumerator.MoveNext())
                    {
                        var current = objectVisibilityEnumerator.Current;
                        var go = current.Key;
                        if (go != null && go.activeInHierarchy && !shouldIgnore(go) && current.Value >= MinimumVisibility)
                        {
                            detectedObjects.Add(go);
                        }
                    }
                }
                else if (DetectionMode == SensorMode.RigidBodies)
                {
                    var rigidBodyCollidersEnumerator = rigidBodyColliders.GetEnumerator();
                    while (rigidBodyCollidersEnumerator.MoveNext())
                    {
                        var go = rigidBodyCollidersEnumerator.Current.Key;
                        if (go != null && go.activeInHierarchy && !shouldIgnore(go))
                        {
                            detectedObjects.Add(go);
                        }
                    }
                }
                else
                {
                    var gameObjectCollidersEnumerator = gameObjectColliders.GetEnumerator();
                    while (gameObjectCollidersEnumerator.MoveNext())
                    {
                        var go = gameObjectCollidersEnumerator.Current.Key;
                        if (go != null && go.activeInHierarchy && !shouldIgnore(go))
                        {
                            detectedObjects.Add(go);
                        }
                    }
                }

                return detectedObjects;
            }
        }

        public override List<GameObject> DetectedObjectsOrderedByDistance
        {
            get
            {
                detectedObjectsOrderedByDistance.Clear();
                detectedObjectsOrderedByDistance.AddRange(DetectedObjects);
                distanceComparer.Point = transform.position;
                detectedObjectsOrderedByDistance.Sort(distanceComparer);
                return detectedObjectsOrderedByDistance;
            }
        }

        public Dictionary<GameObject, float> ObjectVisibilities
        {
            get
            {
                return objectVisibility;
            }
        }

        public override float GetVisibility(GameObject go)
        {
            if (!RequiresLineOfSight) return base.GetVisibility(go);
            else if (objectVisibility.ContainsKey(go)) return objectVisibility[go];
            else return 0f;
        }

        public List<Transform> GetVisibleTransforms(GameObject go)
        {
            RayCastTargets targets;
            if (go != null && rayCastTargets.TryGetValue(go, out targets))
            {
                return targets.GetVisibleTransforms();
            }
            else
            {
                return new List<Transform>();
            }
        }

        public List<Vector3> GetVisiblePositions(GameObject go)
        {
            RayCastTargets targets;
            if (go != null && rayCastTargets.TryGetValue(go, out targets))
            {
                return targets.GetVisibleTargetPositions();
            }
            else
            {
                return new List<Vector3>();
            }
        }

        /// <summary>
        /// 被检测到的Collider
        /// </summary>
        Dictionary<GameObject, List<Collider>> gameObjectColliders = new Dictionary<GameObject, List<Collider>>();

        /// <summary>
        /// 被检测到的Rigidbody
        /// </summary>
        Dictionary<GameObject, List<Collider>> rigidBodyColliders = new Dictionary<GameObject, List<Collider>>();

        Dictionary<Collider, GameObject> attachedRigidBody = new Dictionary<Collider, GameObject>();

        Dictionary<GameObject, RayCastTargets> rayCastTargets = new Dictionary<GameObject, RayCastTargets>();

        Dictionary<GameObject, float> objectVisibility = new Dictionary<GameObject, float>();

        List<RayCastResult> raycastResults = new List<RayCastResult>();

        List<GameObject> gameObjectList = new List<GameObject>();

        List<GameObject> detectedObjects = new List<GameObject>();

        List<GameObject> detectedObjectsOrderedByDistance = new List<GameObject>();

        DistanceFromPointComparer distanceComparer = new DistanceFromPointComparer();

        int prevNumberOfRays = 0;

        protected static ListCache<Collider> colliderListCache = new ListCache<Collider>();
        protected static ListCache<Vector3> vector3ListCache = new ListCache<Vector3>();
        class RayCastTargetsCache : ObjectCache<RayCastTargets>
        {
            public override void Dispose(RayCastTargets obj)
            {
                obj.dispose();
                base.Dispose(obj);
            }
        }
        static RayCastTargetsCache rayCastTargetsCache = new RayCastTargetsCache();

        struct RayCastResult
        {
            public GameObject go;
            public Vector3 testPoint;
            public Vector3 obstructionPoint;
            public bool isObstructed;
        }

        class RayCastTargets
        {
            GameObject go;
            Transform[] targetTransforms;
            List<Vector3> targetPoints;
            List<Vector3> returnPoints;
            List<bool> isTargetVisible;

            public RayCastTargets()
            {
                returnPoints = new List<Vector3>();
                isTargetVisible = new List<bool>();
            }

            public bool IsTransforms()
            {
                return targetTransforms != null;
            }

            public void Set(GameObject go, Transform[] targets)
            {
                this.go = go;
                targetTransforms = targets;
                targetPoints = null;
                isTargetVisible.Clear(); 
                for (int i = 0; i < targets.Length; i++) isTargetVisible.Add(false);
            }

            public void Set(GameObject go, List<Vector3> targets)
            {
                this.go = go;
                targetTransforms = null;
                targetPoints = targets;
                isTargetVisible.Clear(); for (int i = 0; i < targets.Count; i++) isTargetVisible.Add(false);
            }

            public List<Transform> GetVisibleTransforms()
            {
                var visibleList = new List<Transform>();
                for (int i = 0; i < isTargetVisible.Count; i++)
                {
                    if (isTargetVisible[i]) visibleList.Add(targetTransforms[i]);
                }
                return visibleList;
            }

            public List<Vector3> GetVisibleTargetPositions()
            {
                var visibleList = new List<Vector3>();
                if (targetTransforms != null)
                {
                    for (int i = 0; i < isTargetVisible.Count; i++)
                    {
                        if (isTargetVisible[i]) visibleList.Add(targetTransforms[i].position);
                    }
                }
                else
                {
                    for (int i = 0; i < isTargetVisible.Count; i++)
                    {
                        if (isTargetVisible[i]) visibleList.Add(go.transform.TransformPoint(targetPoints[i]));
                    }
                }
                return visibleList;
            }

            public void SetIsTargetVisible(int i, bool isVisible)
            {
                isTargetVisible[i] = isVisible;
            }

            public List<Vector3> getTargetPoints()
            {
                returnPoints.Clear();
                if (targetTransforms != null)
                {
                    for (int i = 0; i < targetTransforms.Length; i++)
                    {
                        returnPoints.Add(targetTransforms[i].position);
                    }
                }
                else
                {
                    var go = this.go;
                    for (int i = 0; i < targetPoints.Count; i++)
                    {
                        returnPoints.Add(go.transform.TransformPoint(targetPoints[i]));
                    }
                }
                return returnPoints;
            }

            public void dispose()
            {
                if (targetPoints != null) { vector3ListCache.Dispose(targetPoints); }
            }
        }

        protected virtual void OnEnable()
        {
            clearColliders();
            clearRayCastTargets();
            objectVisibility.Clear();
            raycastResults.Clear();
            gameObjectList.Clear();
        }

        protected GameObject addCollider(Collider c)
        {
            GameObject newColliderDetection = null;
            GameObject newRigidBodyDetection = null;

            if (addColliderToMap(c, c.gameObject, gameObjectColliders))
            {
                disposeRayCastTarget(c.gameObject);
                newColliderDetection = c.gameObject;
            }
            if (c.attachedRigidbody != null) 
            {
                attachedRigidBody[c] = c.attachedRigidbody.gameObject;

                if (addColliderToMap(c, c.attachedRigidbody.gameObject, rigidBodyColliders)) 
                {
                    disposeRayCastTarget(c.attachedRigidbody.gameObject);
                    newRigidBodyDetection = c.attachedRigidbody.gameObject;
                }
            }

            var newDetection = DetectionMode == SensorMode.Colliders ? newColliderDetection : newRigidBodyDetection;
            if (shouldIgnore(newDetection))
            {
                return null;
            }
            else if (RequiresLineOfSight && newDetection != null)
            {
                bool prevDetected = objectVisibility.ContainsKey(newDetection) && objectVisibility[newDetection] >= MinimumVisibility;
                var targets = getRayCastTargets(newDetection);
                if (TestLOSTargetsOnly && !targets.IsTransforms()) return null;
                objectVisibility[newDetection] = testObjectVisibility(newDetection, targets);

                if (!prevDetected && objectVisibility[newDetection] >= MinimumVisibility) return newDetection;
                else return null;
            }
            else
            {
                return newDetection;
            }
        }

        protected GameObject removeCollider(Collider c)
        {
            if (c == null)
            {
                clearDestroyedGameObjects();
                return null;
            }

            GameObject colliderDetectionLost = null;
            GameObject rigidBodyDetectionLost = null;

            if (removeColliderFromMap(c, c.gameObject, gameObjectColliders))
            {
                disposeRayCastTarget(c.gameObject);
                colliderDetectionLost = c.gameObject;
            }

            GameObject rigidBody;
            if (attachedRigidBody.TryGetValue(c, out rigidBody)) 
            {
                if (removeColliderFromMap(c, rigidBody, rigidBodyColliders)) 
                {
                    disposeRayCastTarget(rigidBody);
                    rigidBodyDetectionLost = rigidBody;
                }

                attachedRigidBody.Remove(c);
            }

            var detectionLost = DetectionMode == SensorMode.Colliders ? colliderDetectionLost : rigidBodyDetectionLost;
            if (shouldIgnore(detectionLost))
            {
                return null;
            }
            else if (RequiresLineOfSight && detectionLost != null)
            {
                float visibility;
                if (objectVisibility.TryGetValue(detectionLost, out visibility)) {
                    objectVisibility.Remove(detectionLost);
                    if (visibility >= MinimumVisibility) {
                        return detectionLost;
                    }
                }
                return null;
            }
            else
            {
                return detectionLost;
            }
        }

        protected void clearDestroyedGameObjects()
        {
            gameObjectList.Clear();
            var collidersGameObjectsEnumerator = gameObjectColliders.GetEnumerator();
            while (collidersGameObjectsEnumerator.MoveNext())
            {
                var go = collidersGameObjectsEnumerator.Current.Key;
                if (go == null) { gameObjectList.Add(go); }
            }
            for (int i = 0; i < gameObjectList.Count; i++)
            {
                gameObjectColliders.Remove(gameObjectList[i]);
            }

            gameObjectList.Clear();
            var rigidBodyGameObjectsEnumerator = rigidBodyColliders.GetEnumerator();
            while (rigidBodyGameObjectsEnumerator.MoveNext())
            {
                var go = rigidBodyGameObjectsEnumerator.Current.Key;
                if (go == null) { gameObjectList.Add(go); }
            }
            for (int i = 0; i < gameObjectList.Count; i++)
            {
                var colliderList = rigidBodyColliders[gameObjectList[i]];
                for (int j = 0; j < colliderList.Count; j++) 
                {
                    attachedRigidBody.Remove(colliderList[j]);
                }
                rigidBodyColliders.Remove(gameObjectList[i]);
            }
        }

        protected void clearColliders()
        {
            var collidersEnumerator = gameObjectColliders.GetEnumerator();
            while (collidersEnumerator.MoveNext())
            {
                var colliderList = collidersEnumerator.Current.Value;
                colliderListCache.Dispose(colliderList);
            }
            gameObjectColliders.Clear();

            collidersEnumerator = rigidBodyColliders.GetEnumerator();
            while (collidersEnumerator.MoveNext())
            {
                var colliderList = collidersEnumerator.Current.Value;
                colliderListCache.Dispose(colliderList);
            }
            rigidBodyColliders.Clear();

            attachedRigidBody.Clear();

            clearLineOfSight();
        }

        protected void clearLineOfSight()
        {
            objectVisibility.Clear();
            raycastResults.Clear();
        }

        protected void refreshLineOfSight()
        {
            if (prevNumberOfRays != NumberOfRays)
            {
                prevNumberOfRays = NumberOfRays;
                clearRayCastTargets();
            }

            objectVisibility.Clear();
            raycastResults.Clear();
            if (DetectionMode == SensorMode.RigidBodies)
            {
                var gosEnumerator = rigidBodyColliders.GetEnumerator();
                while (gosEnumerator.MoveNext())
                {
                    var go = gosEnumerator.Current.Key;
                    if (go == null || shouldIgnore(go)) continue;
                    var targets = getRayCastTargets(go);
                    if (TestLOSTargetsOnly && !targets.IsTransforms()) continue;
                    objectVisibility[go] = testObjectVisibility(go, targets);
                }
            }
            else
            {
                var gosEnumerator = gameObjectColliders.GetEnumerator();
                while (gosEnumerator.MoveNext())
                {
                    var go = gosEnumerator.Current.Key;
                    if (go == null || shouldIgnore(go)) continue;
                    var targets = getRayCastTargets(go);
                    if (TestLOSTargetsOnly && !targets.IsTransforms()) continue;
                    objectVisibility[go] = testObjectVisibility(go, targets);
                }
            }
        }

        float testObjectVisibility(GameObject go, RayCastTargets targets)
        {
            int nSuccess = 0;
            var rayCastTargets = getRayCastTargets(go);
            List<Vector3> testPoints = rayCastTargets.getTargetPoints();
            for (int i = 0; i < testPoints.Count; i++)
            {
                var testPoint = testPoints[i];
                Vector3 obstructionPoint;
                var result = new RayCastResult();
                result.go = go;
                result.testPoint = testPoint;
                result.isObstructed = false;
                if (isInLineOfSight(go, testPoint, out obstructionPoint))
                {
                    nSuccess++;
                    rayCastTargets.SetIsTargetVisible(i, true);
                }
                else
                {
                    result.isObstructed = true;
                    result.obstructionPoint = obstructionPoint;
                    rayCastTargets.SetIsTargetVisible(i, false);
                }
                raycastResults.Add(result);
            }

            return nSuccess / (float)testPoints.Count;
        }

        RayCastTargets getRayCastTargets(GameObject go)
        {
            RayCastTargets rts;
            if (rayCastTargets.TryGetValue(go, out rts))
            {
                return rts;
            }
            else
            {
                var losTargets = go.GetComponent<LOSTargets>();
                rts = rayCastTargetsCache.Get();
                if (losTargets != null && losTargets.Targets != null)
                {
                    rts.Set(go, losTargets.Targets);
                    rayCastTargets.Add(go, rts);
                    return rts;
                }
                else
                {
                    rts.Set(go, generateRayCastTargets(go));
                    rayCastTargets.Add(go, rts);
                    return rts;
                }
            }
        }

        List<Vector3> generateRayCastTargets(GameObject go)
        {
            List<Collider> cs;
            if (DetectionMode == SensorMode.Colliders) cs = gameObjectColliders[go];
            else cs = rigidBodyColliders[go];

            List<Vector3> rts = vector3ListCache.Get();
            if (NumberOfRays == 1)
            {
                rts.Add(getCentreOfColliders(go, cs));
            }
            else
            {
                for (int i = 0; i < NumberOfRays; i++)
                {
                    rts.Add(getRandomPointInColliders(go, cs));
                }
            }
            return rts;
        }

        bool isInLineOfSight(GameObject go, Vector3 testPoint, out Vector3 obstructionPoint)
        {
            obstructionPoint = Vector3.zero;
            var toGoCentre = testPoint - transform.position;

            var ray = new Ray(transform.position, toGoCentre.normalized);
            RaycastHit hitInfo;
            if (Physics.Raycast(ray, out hitInfo, toGoCentre.magnitude, BlocksLineOfSight))
            {
                // Ray hit something, check that it was the target.
                if (DetectionMode == SensorMode.RigidBodies && hitInfo.rigidbody != null && hitInfo.rigidbody.gameObject == go)
                {
                    return true;
                }
                else if (DetectionMode == SensorMode.Colliders && hitInfo.collider.gameObject == go)
                {
                    return true;
                }
                else
                {
                    obstructionPoint = hitInfo.point;
                    return false;
                }
            }
            else
            {
                // Ray didn't hit anything so assume target is in line of sight
                return true;
            }
        }

        Vector3 getCentreOfColliders(GameObject goRoot, List<Collider> goColliders)
        {
            Vector3 aggregate = Vector3.zero;
            for (int i = 0; i < goColliders.Count; i++)
            {
                var c = goColliders[i];
                aggregate += c.bounds.center - goRoot.transform.position;
            }
            return aggregate / goColliders.Count;
        }

        Vector3 getRandomPointInColliders(GameObject goRoot, List<Collider> colliders)
        {
            // Choose a random collider weighted by its volume
            Collider rc = colliders[0];
            var totalVolume = 0f;
            for (int i = 0; i < colliders.Count; i++)
            {
                var c = colliders[i];
                totalVolume += c.bounds.size.x * c.bounds.size.y + c.bounds.size.z;
            }

            var r = Random.Range(0f, 1f);
            for (int i = 0; i < colliders.Count; i++)
            {
                var c = colliders[i];
                rc = c;
                var v = c.bounds.size.x * c.bounds.size.y * c.bounds.size.z;
                r -= v / totalVolume;
                if (r <= 0f) break;
            }

            // Now choose a random point within that random collider and return it
            var rp = new Vector3(Random.Range(-.5f, .5f), Random.Range(-.5f, .5f), Random.Range(-.5f, .5f));
            rp.Scale(rc.bounds.size);
            rp += rc.bounds.center - goRoot.transform.position;
            var goScale = goRoot.transform.lossyScale;
            rp.Scale(new Vector3(1 / goScale.x, 1 / goScale.y, 1 / goScale.z));
            return rp;
        }

        bool addColliderToMap(Collider c, GameObject go, Dictionary<GameObject, List<Collider>> dict)
        {
            var newDetection = false;
            List<Collider> colliderList;
            if (!dict.TryGetValue(go, out colliderList))
            {
                newDetection = true;
                colliderList = colliderListCache.Get();
                dict[go] = colliderList;
            }
            if (!colliderList.Contains(c)) {
                colliderList.Add(c);
            }
            return newDetection;
        }

        bool removeColliderFromMap(Collider c, GameObject go, Dictionary<GameObject, List<Collider>> dict)
        {
            var detectionLost = false;
            List<Collider> colliderList;
            if (dict.TryGetValue(go, out colliderList))
            {
                colliderList.Remove(c);
                if (colliderList.Count == 0)
                {
                    detectionLost = true;
                    dict.Remove(go);
                    colliderListCache.Dispose(colliderList);
                }
            }
            return detectionLost;
        }

        void clearRayCastTargets()
        {
            var rayCastTargetsEnumerator = rayCastTargets.GetEnumerator();
            while (rayCastTargetsEnumerator.MoveNext()) 
            {
                var rayCastTarget = rayCastTargetsEnumerator.Current.Value;
                rayCastTargetsCache.Dispose(rayCastTarget);
            }
            rayCastTargets.Clear();
        }

        void disposeRayCastTarget(GameObject forGameObject)
        {
            if (rayCastTargets.ContainsKey(forGameObject))
            {
                rayCastTargetsCache.Dispose(rayCastTargets[forGameObject]);
                rayCastTargets.Remove(forGameObject);
            }
        }

        protected static readonly Color GizmoColor = new Color(51 / 255f, 255 / 255f, 255 / 255f);
        protected static readonly Color GizmoBlockedColor = Color.red;
        public virtual void OnDrawGizmosSelected()
        {
            if (!isActiveAndEnabled) return;

            Gizmos.color = GizmoColor;
            foreach (GameObject go in DetectedObjects)
            {
                Vector3 goCentre = getCentreOfColliders(go, DetectionMode == SensorMode.RigidBodies && rigidBodyColliders.ContainsKey(go)
                    ? rigidBodyColliders[go] : gameObjectColliders[go]) + go.transform.position;
                Gizmos.DrawIcon(goCentre, "SensorToolkit/eye.png", true);
            }

            if (RequiresLineOfSight && ShowRayCastDebug != null)
            {
                foreach (RayCastResult result in raycastResults)
                {
                    if (!ShowRayCastDebug.Contains(result.go)) continue;

                    Gizmos.color = GizmoColor;
                    if (result.isObstructed)
                    {
                        Gizmos.DrawLine(transform.position, result.obstructionPoint);
                        Gizmos.color = GizmoBlockedColor;
                        Gizmos.DrawLine(result.obstructionPoint, result.testPoint);
                        Gizmos.DrawCube(result.testPoint, Vector3.one * 0.1f);
                    }
                    else
                    {
                        Gizmos.DrawLine(transform.position, result.testPoint);
                        Gizmos.DrawCube(result.testPoint, Vector3.one * 0.1f);
                    }
                }
            }
        }
    }
}