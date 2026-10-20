using UnityEngine;

[RequireComponent(typeof(Terrain), typeof(TerrainCollider))]
public class RoverTerrainGenerator : MonoBehaviour
{
    [Min(40f)] public float hillWidth = 100f;
    [Min(1f)] public float maximumHeight = 8f;

    private Terrain terrain;
    private TerrainCollider terrainCollider;
    private TerrainData original;
    private TerrainData generated;
    private float[,] heights;

    public void Generate(int seed, float difficulty, bool flat = false)
    {
        if (generated == null)
        {
            terrain = GetComponent<Terrain>();
            terrainCollider = GetComponent<TerrainCollider>();
            original = terrain.terrainData;
            // Never edit the shared TerrainData asset.
            generated = Instantiate(original);
            generated.name = "Rover runtime hills";
            generated.heightmapResolution = 129;
            generated.size = original.size;
            heights = new float[129, 129];
        }

        var random = new System.Random(seed);
        float offsetX = (float)random.NextDouble() * 1000f;
        float offsetZ = (float)random.NextDouble() * 1000f;
        // Flat practice uses the same safe runtime terrain, without hills.
        float amplitude = flat ? 0f : Mathf.Lerp(1f, maximumHeight, Mathf.Clamp01(difficulty));
        Vector3 size = generated.size;
        for (int z = 0; z < 129; z++)
            for (int x = 0; x < 129; x++)
            {
                float nx = offsetX + x / 128f * size.x / hillWidth;
                float nz = offsetZ + z / 128f * size.z / hillWidth;
                // Broad hills with a small, smooth secondary variation.
                float noise = 0.75f * Mathf.PerlinNoise(nx, nz)
                    + 0.25f * Mathf.PerlinNoise(nx * 2f, nz * 2f);
                heights[z, x] = Mathf.Clamp01((0.25f + amplitude * noise) / size.y);
            }

        generated.SetHeights(0, 0, heights);
        terrain.terrainData = generated;
        terrainCollider.terrainData = generated;
        terrain.Flush();
        Physics.SyncTransforms();
    }

    public void Restore()
    {
        if (original == null)
            return;
        if (terrain != null)
            terrain.terrainData = original;
        if (terrainCollider != null)
            terrainCollider.terrainData = original;
        Physics.SyncTransforms();
    }

    private void OnDestroy()
    {
        Restore();
        if (generated != null)
            Destroy(generated);
    }
}
