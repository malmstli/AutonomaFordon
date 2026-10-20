using System.Collections;
using UnityEngine;

// Scene coordination only. The agents still choose every flight/driving action.
public class FinalDemoCoordinator : MonoBehaviour
{
    public RoverAgent rover;
    public DroneController drone;
    public Camera overviewCamera;

    private IEnumerator Start()
    {
        if (rover == null || drone == null) yield break;
        // Only this combined scene owns the overview camera.
        if (overviewCamera != null)
            foreach (var follow in FindObjectsByType<CameraFollow>(FindObjectsInactive.Include))
                if (follow.camera == overviewCamera.transform) follow.enabled = false;
        rover.EpisodeStarted += MarkObstacles;
        yield return new WaitForFixedUpdate();
        MarkObstacles();

        // Preserve the original policy's start relative to MeetingPoint.
        Vector3 spawn = rover.meetingPoint.position + new Vector3(39.2f, -37.83f, -14.4f);
        drone.transform.position = spawn;
        drone.gameObject.SetActive(true);
    }

    private void MarkObstacles()
    {
        // Both rock and building colliders use the existing drone obstacle tag.
        GameObject root = GameObject.Find("Rover generated obstacles");
        if (root == null) return;
        foreach (Transform part in root.GetComponentsInChildren<Transform>())
            part.gameObject.tag = "Building";
    }

    private void LateUpdate()
    {
        if (overviewCamera == null || rover == null || drone == null) return;
        Bounds view = new Bounds(rover.transform.position, Vector3.one * 12f);
        view.Encapsulate(rover.meetingPoint.position);
        if (drone.gameObject.activeInHierarchy) view.Encapsulate(drone.transform.position);
        overviewCamera.orthographic = true;
        overviewCamera.orthographicSize = Mathf.Max(22f, view.size.magnitude * 0.65f);
        overviewCamera.transform.position = view.center + new Vector3(0f, 100f, -60f);
        overviewCamera.transform.LookAt(view.center);
    }

    private void OnDestroy()
    {
        if (rover != null) rover.EpisodeStarted -= MarkObstacles;
    }
}
