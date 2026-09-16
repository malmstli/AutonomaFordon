using UnityEngine;

public class engine : MonoBehaviour
{
    public float rotationSpeed = 1000.0f;

    private void Update()
    {
        transform.Rotate(0.0f, rotationSpeed * Time.deltaTime, 0.0f, Space.Self);
    }
}
