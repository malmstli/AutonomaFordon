using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform camera;
    public Vector3 offset;
    public bool followYRotation = true;

    private void LateUpdate()
    {
        if (camera == null)
        {
            return;
        }

        camera.position = transform.position + offset;

        float yRotation = followYRotation ? transform.eulerAngles.y : camera.eulerAngles.y;
        camera.rotation = Quaternion.Euler(0.0f, yRotation, 0.0f);
    }
}
