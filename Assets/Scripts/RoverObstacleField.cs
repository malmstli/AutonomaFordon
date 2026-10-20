using System.Collections.Generic;
using UnityEngine;

// Owns only the rover's generated obstacles, never the shared prefab assets.
public class RoverObstacleField : MonoBehaviour
{
    public GameObject[] templates;
    [Range(8, 31)] public int obstacleLayer = 29;
    [Min(2f)] public float rockWidth = 5f;
    [Min(2f)] public float buildingWidth = 8f;
    [Min(2f)] public float passageClearance = 8f;
    private readonly List<GameObject> objects = new List<GameObject>();
    private readonly List<Bounds> bounds = new List<Bounds>();
    private readonly HashSet<Collider> colliders = new HashSet<Collider>();
    private readonly Collider[] nearColliders = new Collider[16];
    private GameObject generatedRoot;
    public int Count => objects.Count;
    public int LayerMask => 1 << obstacleLayer;
    private static readonly Vector2[] FixedLayout = {
        Vector2.zero, new Vector2(-0.22f, 0.12f), new Vector2(0.18f, 0.22f),
        new Vector2(0.05f, -0.22f), new Vector2(-0.28f, -0.22f), new Vector2(0.3f, 0.1f)
    };

    public void Clear()
    {
        if (generatedRoot != null)
        {
            generatedRoot.SetActive(false); // Remove old colliders before another reset.
            if (Application.isPlaying) Destroy(generatedRoot);
            else DestroyImmediate(generatedRoot);
        }
        generatedRoot = null;
        objects.Clear(); bounds.Clear(); colliders.Clear();
    }

    public bool Generate(Collider surface, Vector3 center, Vector2 area, int seed, bool randomized, int count, float boundaryMargin = 35f)
    {
        Clear();
        if (count <= 0) return true;
        if (surface == null || templates == null || templates.Length == 0) return false;
        // Use the same clipped training rectangle as the rover's spawn sampler.
        Bounds terrainBounds = surface.bounds;
        float margin = Mathf.Max(10f, boundaryMargin);
        float minX = Mathf.Max(center.x - area.x * 0.5f, terrainBounds.min.x + margin);
        float maxX = Mathf.Min(center.x + area.x * 0.5f, terrainBounds.max.x - margin);
        float minZ = Mathf.Max(center.z - area.y * 0.5f, terrainBounds.min.z + margin);
        float maxZ = Mathf.Min(center.z + area.y * 0.5f, terrainBounds.max.z - margin);
        if (maxX <= minX || maxZ <= minZ) return false;
        center.x = (minX + maxX) * 0.5f; center.z = (minZ + maxZ) * 0.5f;
        area = new Vector2(maxX - minX, maxZ - minZ);
        generatedRoot = new GameObject("Rover generated obstacles");
        var random = new System.Random(seed);
        for (int index = 0; index < count; index++)
        {
            GameObject template = templates[index % templates.Length];
            if (template == null) { Clear(); return false; }
            GameObject item = Instantiate(template, generatedRoot.transform);
            item.name = "Rover obstacle " + index;
            item.SetActive(true);
            foreach (Transform part in item.GetComponentsInChildren<Transform>(true)) part.gameObject.layer = obstacleLayer;
            foreach (Rigidbody body in item.GetComponentsInChildren<Rigidbody>()) body.isKinematic = true;
            if (item.GetComponentsInChildren<Collider>().Length == 0)
            {
                Bounds visible = VisibleBounds(item);
                BoxCollider box = item.AddComponent<BoxCollider>();
                box.center = item.transform.InverseTransformPoint(visible.center);
                box.size = new Vector3(visible.size.x / Mathf.Abs(item.transform.lossyScale.x),
                    visible.size.y / Mathf.Abs(item.transform.lossyScale.y), visible.size.z / Mathf.Abs(item.transform.lossyScale.z));
            }
            foreach (Collider collider in item.GetComponentsInChildren<Collider>())
            {
                collider.enabled = true;
                collider.isTrigger = false;
            }
            Physics.SyncTransforms();
            Bounds initial = SolidBounds(item);
            float width = template.name.ToLowerInvariant().Contains("rock") ? rockWidth : buildingWidth;
            item.transform.localScale *= width / Mathf.Max(0.1f, initial.size.x, initial.size.z);
            bool placed = false;
            for (int attempt = 0; attempt < (randomized ? 128 : 1); attempt++)
            {
                Vector2 offset = randomized
                    ? new Vector2((float)random.NextDouble() * 0.7f - 0.35f, (float)random.NextDouble() * 0.7f - 0.35f)
                    : FixedLayout[index % FixedLayout.Length];
                Vector3 point = center + new Vector3(offset.x * area.x, 0f, offset.y * area.y);
                var ray = new Ray(new Vector3(point.x, surface.bounds.max.y + 20f, point.z), Vector3.down);
                if (!surface.Raycast(ray, out RaycastHit hit, surface.bounds.size.y + 40f)
                    || Vector3.Angle(hit.normal, Vector3.up) > 15f) continue;
                item.transform.SetPositionAndRotation(hit.point,
                    Quaternion.Euler(0f, randomized ? (float)random.NextDouble() * 360f : index * 47f, 0f));
                Physics.SyncTransforms();
                Bounds candidate = SolidBounds(item);
                item.transform.position += Vector3.up * (hit.point.y - candidate.min.y);
                Physics.SyncTransforms();
                candidate = SolidBounds(item);
                if (candidate.min.x < surface.bounds.min.x + 10f || candidate.max.x > surface.bounds.max.x - 10f
                    || candidate.min.z < surface.bounds.min.z + 10f || candidate.max.z > surface.bounds.max.z - 10f) continue;
                bool clear = true;
                foreach (Bounds existing in bounds)
                    if (Mathf.Abs(candidate.center.x - existing.center.x) < candidate.extents.x + existing.extents.x + passageClearance
                        && Mathf.Abs(candidate.center.z - existing.center.z) < candidate.extents.z + existing.extents.z + passageClearance)
                    { clear = false; break; }
                if (!clear) continue;
                objects.Add(item); bounds.Add(candidate);
                foreach (Collider collider in item.GetComponentsInChildren<Collider>()) colliders.Add(collider);
                placed = true;
                break;
            }
            if (!placed)
            {
                Debug.LogError("Obstacle placement failed: index=" + index + "; randomized=" + randomized
                    + "; center=" + center + "; area=" + area + "; terrain=" + surface.bounds, this);
                Clear(); return false;
            }
        }
        Physics.SyncTransforms();
        return true;
    }

    public bool IsObstacle(Collider collider) => collider != null && colliders.Contains(collider);
    public Vector3 Anchor(int index) => bounds[index % bounds.Count].center;

    public bool IsClear(Vector3 point, float clearance)
    {
        foreach (Bounds obstacle in bounds)
        {
            float dx = Mathf.Max(0f, Mathf.Abs(point.x - obstacle.center.x) - obstacle.extents.x);
            float dz = Mathf.Max(0f, Mathf.Abs(point.z - obstacle.center.z) - obstacle.extents.z);
            if (dx * dx + dz * dz < clearance * clearance) return false;
        }
        return true;
    }

    public bool BlocksLine(Vector3 start, Vector3 target)
    {
        Vector3 direction = target - start;
        return Physics.SphereCast(start + Vector3.up * 1.5f, 1.5f, direction.normalized,
            out _, direction.magnitude, LayerMask, QueryTriggerInteraction.Ignore);
    }

    public float ReadDistance(Vector3 origin, Vector3 direction, float range, float radius = 2f)
    {
        // SphereCast alone misses colliders already touching its starting sphere.
        int count = Physics.OverlapSphereNonAlloc(origin, radius, nearColliders, LayerMask, QueryTriggerInteraction.Ignore);
        if (count == nearColliders.Length) return 0f;
        for (int i = 0; i < count; i++)
        {
            Vector3 offset = nearColliders[i].bounds.ClosestPoint(origin) - origin;
            if (offset.sqrMagnitude < 0.0001f || Vector3.Dot(offset.normalized, direction) > 0.25f) return 0f;
        }
        return Physics.SphereCast(origin, radius, direction, out RaycastHit hit, range, LayerMask, QueryTriggerInteraction.Ignore)
            ? Mathf.Clamp01(hit.distance / range) : 1f;
    }

    private static Bounds SolidBounds(GameObject item)
    {
        Collider[] parts = item.GetComponentsInChildren<Collider>();
        Bounds result = parts[0].bounds;
        for (int index = 1; index < parts.Length; index++) result.Encapsulate(parts[index].bounds);
        return result;
    }

    private static Bounds VisibleBounds(GameObject item)
    {
        Renderer[] parts = item.GetComponentsInChildren<Renderer>();
        if (parts.Length == 0) return new Bounds(item.transform.position, Vector3.one * 3f);
        Bounds result = parts[0].bounds;
        for (int index = 1; index < parts.Length; index++) result.Encapsulate(parts[index].bounds);
        return result;
    }

    private void OnDestroy() { Clear(); }
}
