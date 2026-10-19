using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(BehaviorParameters))]
[RequireComponent(typeof(DecisionRequester))]
public class DroneController : Agent
{
    public float thrustChangeRate = 10.0f;
    public float maximumThrustOffset = 20.0f;
    public float thrustReturnRate = 10.0f;
    public float maximumAltitudeAboveStart = 30.0f;
    public float altitudeBrakeStrength = 3.0f;
    public float minimumTerrainClearance = 2.0f;
    public float terrainEdgeMargin = 2.0f;
    public float tiltChangeRate = 30.0f;
    public float tiltStabilizationTorque = 20.0f;
    public float angularDamping = 5.0f;
    public float maximumTiltAngle = 40.0f;
    public float minimumCosine = 0.1f;
    private Transform meetingPoint;
    private Transform landingPoint;
    private Transform landingTruckRoot;
    public float meetingPointRadius = 2.0f;
    public float landingRadius = 1.5f;
    public float maximumLandingSpeed = 1.0f;
    public float maximumLandingTiltAngle = 15.0f;
    public float maximumLandingAngularSpeed = 1.0f;
    public float observationDistance = 50.0f;
    public float progressRewardScale = 0.01f;
    public float retreatPenaltyScale = 0.0001f;
    public float waypointReward = 0.25f;
    public float stepPenalty = -0.0005f;
    public float crashPenalty = -1.0f;
    public float landingReward = 1.0f;

    private Rigidbody rb;
    private Terrain terrain;
    private TerrainCollider terrainCollider;
    private float thrustOffset;
    private float targetPitch;
    private float targetRoll;
    private float requestedThrust;
    private float requestedPitch;
    private float requestedRoll;
    private bool reachedMeetingPoint;
    private bool episodeEnded;
    private float previousTargetDistance;
    private Vector3 startingPosition;
    private Quaternion startingRotation;

    protected override void Awake()
    {
        base.Awake();
        rb = GetComponent<Rigidbody>();
        rb.constraints |= RigidbodyConstraints.FreezeRotationY;
        GameObject terrainObject = GameObject.FindGameObjectWithTag("Floor");
        terrain = terrainObject == null ? null : terrainObject.GetComponent<Terrain>();
        terrainCollider = terrainObject == null ? null : terrainObject.GetComponent<TerrainCollider>();
        startingPosition = rb.position;
        startingRotation = rb.rotation;
        ResolveObjectiveReferences();

        if (terrain == null || terrainCollider == null)
        {
            Debug.LogError("SampleScene must contain a Terrain object tagged Floor.", this);
        }

        if (meetingPoint == null || landingPoint == null || landingTruckRoot == null)
        {
            Debug.LogError("SampleScene must contain objects tagged MeetingPoint and DroneGoal under Truck.", this);
        }
    }

    private void ResolveObjectiveReferences()
    {
        GameObject meetingPointObject = GameObject.FindGameObjectWithTag("MeetingPoint");
        GameObject droneGoalObject = GameObject.FindGameObjectWithTag("DroneGoal");
        GameObject truckObject = GameObject.FindGameObjectWithTag("truck");

        meetingPoint = meetingPointObject == null ? null : meetingPointObject.transform;
        landingPoint = droneGoalObject == null ? null : droneGoalObject.transform;
        landingTruckRoot = truckObject == null ? null : truckObject.transform;

        if (landingPoint != null && landingTruckRoot != null && !landingPoint.IsChildOf(landingTruckRoot))
        {
            Debug.LogError("The DroneGoal object must be a child of the object tagged truck.", this);
            landingPoint = null;
        }
    }

    private void FixedUpdate()
    {
        thrustOffset = Mathf.MoveTowards(
            thrustOffset,
            requestedThrust * maximumThrustOffset,
            thrustChangeRate * Time.fixedDeltaTime);
        targetPitch = Mathf.MoveTowards(
            targetPitch,
            requestedPitch * maximumTiltAngle,
            tiltChangeRate * Time.fixedDeltaTime);
        targetRoll = Mathf.MoveTowards(
            targetRoll,
            requestedRoll * maximumTiltAngle,
            tiltChangeRate * Time.fixedDeltaTime);

        ApplyThrust();
        ApplyTiltTorque();
        LimitTiltAngle();
        EnforceTerrainSafety();
        UpdateObjectiveReward();
    }

    private void EnforceTerrainSafety()
    {
        if (terrain == null || terrainCollider == null)
        {
            return;
        }

        Bounds terrainBounds = terrainCollider.bounds;
        float margin = Mathf.Max(terrainEdgeMargin, 0.0f);
        float minX = terrainBounds.min.x + margin;
        float maxX = terrainBounds.max.x - margin;
        float minZ = terrainBounds.min.z + margin;
        float maxZ = terrainBounds.max.z - margin;
        Vector3 position = rb.position;
        Vector3 velocity = rb.linearVelocity;

        position.x = Mathf.Clamp(position.x, minX, maxX);
        position.z = Mathf.Clamp(position.z, minZ, maxZ);

        Ray terrainRay = new Ray(
            new Vector3(position.x, terrainBounds.max.y + 1.0f, position.z),
            Vector3.down);
        if (terrainCollider.Raycast(terrainRay, out RaycastHit terrainHit, terrainBounds.size.y + 1.0f))
        {
            float minimumHeight = terrainHit.point.y + minimumTerrainClearance;
            if (position.y < minimumHeight)
            {
                position.y = minimumHeight;
                if (velocity.y < 0.0f)
                {
                    velocity.y = 0.0f;
                }
            }
        }

        float fixedDeltaTime = Time.fixedDeltaTime;
        velocity.x = Mathf.Clamp(velocity.x, (minX - position.x) / fixedDeltaTime, (maxX - position.x) / fixedDeltaTime);
        velocity.z = Mathf.Clamp(velocity.z, (minZ - position.z) / fixedDeltaTime, (maxZ - position.z) / fixedDeltaTime);

        rb.position = position;
        rb.linearVelocity = velocity;
    }

    private void ApplyThrust()
    {
        bool altitudeLimitReached = rb.position.y >= startingPosition.y + maximumAltitudeAboveStart;
        float cosine = Vector3.Dot(transform.up.normalized, Vector3.up);
        cosine = Mathf.Max(cosine, minimumCosine);

        float hoverThrust = (rb.mass * Physics.gravity.magnitude) / cosine;
        float allowedThrustOffset = altitudeLimitReached ? Mathf.Min(thrustOffset, 0.0f) : thrustOffset;
        float thrustForce = hoverThrust + allowedThrustOffset;

        rb.AddForce(transform.up * thrustForce, ForceMode.Force);

        if (altitudeLimitReached && rb.linearVelocity.y > 0.0f)
        {
            rb.AddForce(Vector3.down * rb.linearVelocity.y * altitudeBrakeStrength, ForceMode.Acceleration);
        }
    }

    private void ApplyTiltTorque()
    {
        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
        Vector3 localAngles = GetLocalTiltAngles();

        Vector3 localTorque = new Vector3(
            (targetPitch - localAngles.x) * tiltStabilizationTorque -
            localAngularVelocity.x * angularDamping,
            0.0f,
            (targetRoll - localAngles.z) * tiltStabilizationTorque -
            localAngularVelocity.z * angularDamping);

        rb.AddRelativeTorque(localTorque, ForceMode.Force);
    }

    public override void OnEpisodeBegin()
    {
        rb.position = startingPosition;
        rb.rotation = startingRotation;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        requestedThrust = 0.0f;
        requestedPitch = 0.0f;
        requestedRoll = 0.0f;
        thrustOffset = 0.0f;
        targetPitch = 0.0f;
        targetRoll = 0.0f;
        reachedMeetingPoint = false;
        episodeEnded = false;
        previousTargetDistance = GetTargetDistance();
    }

    private Vector3 GetLocalTiltAngles()
    {
        Quaternion localRotation = transform.parent == null
            ? rb.rotation
            : Quaternion.Inverse(transform.parent.rotation) * rb.rotation;

        Vector3 localAngles = localRotation.eulerAngles;
        localAngles.x = NormalizeAngle(localAngles.x);
        localAngles.z = NormalizeAngle(localAngles.z);
        return localAngles;
    }

    private void LimitTiltAngle()
    {
        Quaternion localRotation = transform.parent == null
            ? rb.rotation
            : Quaternion.Inverse(transform.parent.rotation) * rb.rotation;

        Vector3 localAngles = localRotation.eulerAngles;
        localAngles.x = NormalizeAngle(localAngles.x);
        localAngles.z = NormalizeAngle(localAngles.z);

        float limitedX = Mathf.Clamp(localAngles.x, -maximumTiltAngle, maximumTiltAngle);
        float limitedZ = Mathf.Clamp(localAngles.z, -maximumTiltAngle, maximumTiltAngle);

        if (Mathf.Approximately(localAngles.x, limitedX) &&
            Mathf.Approximately(localAngles.z, limitedZ))
        {
            return;
        }

        Quaternion limitedLocalRotation = Quaternion.Euler(
            limitedX,
            localRotation.eulerAngles.y,
            limitedZ);

        rb.MoveRotation(transform.parent == null
            ? limitedLocalRotation
            : transform.parent.rotation * limitedLocalRotation);
    }

    private float NormalizeAngle(float angle)
    {
        return angle > 180.0f ? angle - 360.0f : angle;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float tiltScale = Mathf.Max(maximumTiltAngle, 0.01f);
        float thrustScale = Mathf.Max(maximumThrustOffset, 0.01f);
        Vector3 localAngles = GetLocalTiltAngles();
        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
        Vector3 localVelocity = transform.InverseTransformDirection(rb.linearVelocity);

        sensor.AddObservation(Mathf.Clamp(localAngles.x / tiltScale, -1.0f, 1.0f));
        sensor.AddObservation(Mathf.Clamp(localAngles.z / tiltScale, -1.0f, 1.0f));
        sensor.AddObservation(localAngularVelocity);
        sensor.AddObservation(localVelocity);
        sensor.AddObservation(thrustOffset / thrustScale);
        sensor.AddObservation(targetPitch / tiltScale);
        sensor.AddObservation(targetRoll / tiltScale);

        Transform target = GetCurrentTarget();
        Vector3 targetOffset = target == null
            ? Vector3.zero
            : transform.InverseTransformDirection(target.position - transform.position);
        float targetScale = Mathf.Max(observationDistance, 0.01f);
        sensor.AddObservation(Vector3.ClampMagnitude(targetOffset / targetScale, 1.0f));
        sensor.AddObservation(reachedMeetingPoint ? 1.0f : 0.0f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        ActionSegment<float> continuousActions = actions.ContinuousActions;
        requestedThrust = Mathf.Clamp(continuousActions[0], -1.0f, 1.0f);
        requestedPitch = Mathf.Clamp(continuousActions[1], -1.0f, 1.0f);
        requestedRoll = Mathf.Clamp(continuousActions[2], -1.0f, 1.0f);
    }

    private void UpdateObjectiveReward()
    {
        if (episodeEnded || meetingPoint == null || landingPoint == null || landingTruckRoot == null)
        {
            return;
        }

        AddReward(stepPenalty);

        if (!reachedMeetingPoint && Vector3.Distance(transform.position, meetingPoint.position) <= meetingPointRadius)
        {
            reachedMeetingPoint = true;
            AddReward(waypointReward);
            previousTargetDistance = Vector3.Distance(transform.position, landingPoint.position);
            return;
        }

        float currentDistance = GetTargetDistance();
        float distanceProgress = previousTargetDistance - currentDistance;
        float rewardScale = !reachedMeetingPoint && distanceProgress < 0.0f
            ? retreatPenaltyScale
            : progressRewardScale;
        AddReward(Mathf.Clamp(distanceProgress * rewardScale, -0.05f, 0.05f));
        previousTargetDistance = currentDistance;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (episodeEnded || meetingPoint == null || landingPoint == null || landingTruckRoot == null)
        {
            return;
        }

        if (!IsLandingSurface(collision.collider) || !reachedMeetingPoint)
        {
            EndEpisodeWithReward(crashPenalty);
            return;
        }

        Rigidbody otherBody = collision.rigidbody;
        Vector3 otherVelocity = otherBody == null ? Vector3.zero : otherBody.linearVelocity;
        Vector3 relativeVelocity = rb.linearVelocity - otherVelocity;
        float tiltAngle = Vector3.Angle(transform.up, Vector3.up);
        bool insideLandingZone = Vector3.Distance(rb.worldCenterOfMass, landingPoint.position) <= landingRadius;
        bool landedSafely = insideLandingZone &&
            relativeVelocity.magnitude <= maximumLandingSpeed &&
            rb.angularVelocity.magnitude <= maximumLandingAngularSpeed &&
            tiltAngle <= maximumLandingTiltAngle;

        EndEpisodeWithReward(landedSafely ? landingReward : crashPenalty);
    }

    private bool IsLandingSurface(Collider other)
    {
        return other.transform.root == landingTruckRoot;
    }

    private void EndEpisodeWithReward(float reward)
    {
        episodeEnded = true;
        AddReward(reward);
        EndEpisode();
    }

    private Transform GetCurrentTarget()
    {
        return reachedMeetingPoint ? landingPoint : meetingPoint;
    }

    private float GetTargetDistance()
    {
        Transform target = GetCurrentTarget();
        return target == null ? 0.0f : Vector3.Distance(transform.position, target.position);
    }

}