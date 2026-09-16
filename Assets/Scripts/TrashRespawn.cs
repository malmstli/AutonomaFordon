using UnityEngine;
using System;

public class TrashRespawn : MonoBehaviour
{
    [SerializeField] private Collider floorCollider;
    private System.Random randomGenerator;

    public void Respawn()
    {
        if (floorCollider == null)
        {
            Debug.LogError("TrashRespawn needs a floor collider assigned.", this);
            return;
        }

        // Initalisera randomGenerator här för att säkerställa att den alltid är klar
        randomGenerator = new System.Random(this.GetHashCode() ^ (int)(Time.time * 10000));

        Bounds floorBounds = floorCollider.bounds;

        for (int attempt = 0; attempt < 20; attempt++)
        {
            Vector3 rayStart = new Vector3(
                Mathf.Lerp(floorBounds.min.x, floorBounds.max.x, (float)randomGenerator.NextDouble()),
                floorBounds.max.y + 10f,
                Mathf.Lerp(floorBounds.min.z, floorBounds.max.z, (float)randomGenerator.NextDouble())
            );

            if (floorCollider.Raycast(new Ray(rayStart, Vector3.down), out RaycastHit hit, 100f))
            {
                float halfHeight = GetComponent<Collider>().bounds.extents.y;
                transform.position = hit.point + Vector3.up * halfHeight;
                transform.rotation = Quaternion.Euler(0f, (float)randomGenerator.NextDouble() * 360f, 0f);
                return;
            }
        }

        Debug.LogWarning("Could not find a valid random position on the floor.", this);
    }

    private void OnCollisionEnter(Collision collision)
    {
        CleanerController cleaner = collision.gameObject.GetComponentInParent<CleanerController>();
        if (cleaner != null)
        {
            cleaner.CollectTrash();
        }
    }
}
