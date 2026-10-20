using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RoverAgent : Agent
{

    // References to the car and target
    public CarControl carControl;
    public Transform meetingPoint;

    public enum RoverLevel { Legacy, Level1, Level2, Level3, Level4, Level5 }
    [Header("Final environment (Legacy preserves the original policy)")]
    public RoverLevel level = RoverLevel.Legacy;
    public bool trainAcrossLevels;
    public bool obstacleObservations;
    public RoverObstacleField obstacleField;
    [Range(1, 6)] public int fixedObstacleCount = 6;
    public bool resetAfterSuccess = true;
    [Min(10f)] public float sensorRange = 30f;
    private static readonly float[] SensorAngles = { -90, -60, -35, -15, 0, 15, 35, 60, 90, 180 };
    private int currentLevel, currentStage;
    private int? nextEpisodeSeed;
    private bool obstacleContact, parked, requireBlockedStart;
    private bool FinalEnvironment => level != RoverLevel.Legacy;
    public int CurrentLevel => currentLevel;
    public bool PauseAfterEpisode { get; set; }
    public bool EvaluationSettings { get; set; }
    public bool InitialPathBlocked { get; private set; }
    public bool ReadyForLanding => parked && GetTargetDistance() < targetDistance
        && rb.linearVelocity.magnitude < stopSpeed && rb.angularVelocity.magnitude < 0.2f && carControl.IsGrounded;
    public event System.Action<bool, string, float, float, float, float> EpisodeFinished;
    public event System.Action EpisodeStarted;
    public void SetNextEpisodeSeed(int seed) { nextEpisodeSeed = seed; }

    private Rigidbody rb;

    // Settings for reaching the target
    public float targetDistance = 5f;
    public float stopSpeed = 0.5f;
    [Min(0.1f)] public float stopDuration = 1f;
    [Min(1f)] public float distanceScale = 100f;
    [Min(0.1f)] public float cruiseSpeed = 6f;
    [Min(0.1f)] public float brakingDeceleration = 2f;
    [Min(1f)] public float maxEpisodeSeconds = 60f;
    [Min(0f)] public float noProgressTimeout = 15f;

    [Header("Level 2 spawning")]
    public bool randomizeSpawns = false;
    public Collider spawnSurface;
    public Vector2 spawnAreaSize = new Vector2(160f, 160f);
    [Min(15f)] public float minTargetDistance = 25f;
    [Min(15f)] public float maxTargetDistance = 60f;
    [Range(0f, 1f)] public float spawnDifficulty = 1f;
    public int spawnSeed = 12345;
    [Min(0f)] public float spawnClearance = 0.03f;
    [Min(6f)] public float spawnBoundaryMargin = 35f;
    [Min(1f)] public float level2EpisodeSeconds = 90f;
    [Min(0f)] public float level2NoProgressTimeout = 30f;

    [Header("Level 3 terrain")]
    public bool proceduralTerrain = false;
    public RoverTerrainGenerator terrainGenerator;
    [Range(0f, 1f)] public float hillDifficulty = 0f;
    [Range(1f, 20f)] public float maximumTerrainSlope = 12f;
    private bool hillyEpisode;
    private float targetGroundOffset;

    private float previousDistance;
    private float bestDistance;
    private float progressCheckpoint;
    private float stoppedTime;
    private float elapsedTime;
    private float noProgressTime;
    private float flippedTime;
    private float airborneTime;
    private bool episodeActive;
    private bool randomizedEpisode;
    private float episodeDifficulty;
    private float initialDistance;
    private float initialHeadingError;
    private System.Random spawnRandom;
    private BoxCollider chassisCollider;
    private readonly Collider[] spawnOverlaps = new Collider[32];

    // Save the rover's starting position and rotation
    private Vector3 startPosition;
    private Quaternion startRotation;
    private Vector3 startTargetPosition;

    // Get the car's Rigidbody and enable AI control
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        carControl.useAI = true;
        // Remember the starting position for episode resets
        startPosition = transform.position;
        startRotation = transform.rotation;
        startTargetPosition = meetingPoint.position;
        spawnRandom = new System.Random(spawnSeed);
        chassisCollider = GetComponent<BoxCollider>();
        targetGroundOffset = TryGround(startTargetPosition, out Vector3 ground)
            ? startTargetPosition.y - ground.y : 0f;
    }

    // Calculate horizontal distance to the target
    private float GetTargetDistance()
    {
        Vector3 direction = meetingPoint.position - rb.position;
        direction.y = 0f;

        return direction.magnitude;
    }

    // Reset the reward tracking at the start of each episode
    public override void OnEpisodeBegin()
    {
        episodeActive = false;
        var parameters = Academy.Instance.EnvironmentParameters;
        parked = obstacleContact = false;
        if (nextEpisodeSeed.HasValue)
        {
            spawnRandom = new System.Random(nextEpisodeSeed.Value);
            nextEpisodeSeed = null;
        }
        if (FinalEnvironment && obstacleField != null) obstacleField.Clear();
        randomizedEpisode = parameters.GetWithDefault("rover_level2", randomizeSpawns ? 1f : 0f) > 0.5f;
        hillyEpisode = parameters.GetWithDefault("rover_level3", proceduralTerrain ? 1f : 0f) > 0.5f;
        randomizedEpisode |= hillyEpisode;
        episodeDifficulty = Mathf.Clamp01(parameters.GetWithDefault("rover_spawn_difficulty", spawnDifficulty));
        if (FinalEnvironment)
        {
            ConfigureFinalEpisode();
            randomizedEpisode = currentLevel >= 2;
            hillyEpisode = currentLevel >= 3;
            episodeDifficulty = currentStage == 0 ? 0.5f : 1f;
        }
        Vector3 position = startPosition;
        Quaternion rotation = startRotation;
        Vector3 targetPosition = startTargetPosition;

        // Stop the previous episode before searching or teleporting.
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        carControl.ResetControls();
        if (hillyEpisode || FinalEnvironment)
        {
            if (terrainGenerator == null || spawnSurface == null
                || terrainGenerator.GetComponent<TerrainCollider>() != spawnSurface)
            {
                Debug.LogError("Level 3 needs a generator on the assigned spawn terrain.", this);
                enabled = false;
                return;
            }
            terrainGenerator.Generate(spawnRandom.Next(),
                FinalEnvironment && EvaluationSettings ? hillDifficulty : FinalEnvironment && trainAcrossLevels && Academy.Instance.IsCommunicatorOn
                    ? Mathf.Min(parameters.GetWithDefault("rover_hill_difficulty", hillDifficulty), currentStage <= 2 ? 0.25f : 1f)
                    : parameters.GetWithDefault("rover_hill_difficulty", hillDifficulty),
                FinalEnvironment && !hillyEpisode);
        }
        else if (terrainGenerator != null)
            terrainGenerator.Restore();
        if (FinalEnvironment && currentLevel >= 4)
        {
            int count = currentLevel == 4 ? (currentStage == 2 ? 1 : fixedObstacleCount)
                : spawnRandom.Next(3, currentStage == 4 ? 7 : 11);
            if (obstacleField == null || !obstacleObservations
                || !obstacleField.Generate(spawnSurface, startPosition, spawnAreaSize, spawnRandom.Next(), currentLevel == 5, count, spawnBoundaryMargin))
            {
                Debug.LogError("Final obstacle level needs a valid obstacle field, sensors and placement.", this);
                enabled = false;
                return;
            }
        }
        bool placed = randomizedEpisode ? TryChooseSpawn(out position, out rotation, out targetPosition)
            : !FinalEnvironment || TryFixedSpawn(out position, out rotation, out targetPosition);
        if (!placed)
        {
            // Fail visibly instead of training on an unsafe or fixed fallback spawn.
            Debug.LogError("No safe rover spawn. Check the ground collider, hills and spawn area.", this);
            enabled = false;
            return;
        }
        meetingPoint.position = targetPosition;

        // Update the Transform too, so observations immediately see the reset pose.
        transform.SetPositionAndRotation(position, rotation);
        rb.position = position;
        rb.rotation = rotation;

        // Reset movement and rotation speed
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Reset wheel spin and smoothed inputs at the new pose.
        carControl.ResetControls();
        Physics.SyncTransforms();
        rb.WakeUp();

        // Reset all timers in simulated seconds, not action callback counts.
        previousDistance = GetTargetDistance();
        initialDistance = previousDistance;
        Vector3 initialDirection = meetingPoint.position - rb.position;
        initialDirection.y = 0f;
        initialHeadingError = Vector3.SignedAngle(rb.rotation * Vector3.forward, initialDirection, Vector3.up);
        bestDistance = previousDistance;
        progressCheckpoint = previousDistance;
        stoppedTime = 0f;
        elapsedTime = 0f;
        noProgressTime = 0f;
        flippedTime = 0f;
        airborneTime = 0f;
        episodeActive = true;
        InitialPathBlocked = FinalEnvironment && obstacleField != null && obstacleField.Count > 0
            && TryGround(position, out Vector3 routeStart) && TryGround(targetPosition, out Vector3 routeGoal)
            && obstacleField.BlocksLine(routeStart, routeGoal);
        EpisodeStarted?.Invoke();
    }

    private void ConfigureFinalEpisode()
    {
        currentStage = trainAcrossLevels && Academy.Instance.IsCommunicatorOn
            ? Mathf.Clamp(Mathf.RoundToInt(Academy.Instance.EnvironmentParameters.GetWithDefault("rover_stage", 0f)), 0, 5) : 5;
        currentLevel = (int)level;
        if (trainAcrossLevels && Academy.Instance.IsCommunicatorOn)
        {
            float draw = Sample(0f, 1f);
            if (currentStage <= 1) currentLevel = draw < 0.1f ? 1 : draw < 0.4f ? 2 : 3;
            else if (currentStage <= 3) currentLevel = draw < 0.15f ? 2 : draw < 0.3f ? 3 : 4;
            else if (currentStage == 4) currentLevel = draw < 0.1f ? 2 : draw < 0.2f ? 3 : draw < 0.35f ? 4 : 5;
            else currentLevel = draw < 0.05f ? 1 : draw < 0.2f ? 2 : draw < 0.4f ? 3 : draw < 0.65f ? 4 : 5;
        }
        requireBlockedStart = currentLevel >= 4 && Sample(0f, 1f) < 0.6f;
    }

    private bool TryFixedSpawn(out Vector3 position, out Quaternion rotation, out Vector3 target)
    {
        position = startPosition; rotation = startRotation; target = startTargetPosition;
        if (!TryGround(startPosition, out Vector3 ground) || !TryGround(startTargetPosition, out Vector3 goal)) return false;
        float rootHeight = float.MinValue;
        foreach (Transform wheelTransform in carControl.wheels)
        {
            WheelCollider wheel = wheelTransform.GetComponent<WheelCollider>();
            float centerHeight = transform.InverseTransformPoint(wheel.transform.TransformPoint(wheel.center)).y * transform.lossyScale.y;
            rootHeight = Mathf.Max(rootHeight, wheel.radius + wheel.suspensionDistance * (1f - wheel.suspensionSpring.targetPosition) - centerHeight);
        }
        rootHeight += spawnClearance;
        rotation = Quaternion.Euler(0f, startRotation.eulerAngles.y, 0f);
        position = ground + Vector3.up * rootHeight;
        target = goal + Vector3.up * targetGroundOffset;
        return IsSafePose(ground, rotation, rootHeight);
    }

    private float Sample(float minimum, float maximum)
    {
        return Mathf.Lerp(minimum, maximum, (float)spawnRandom.NextDouble());
    }

    private bool TryGround(Vector3 point, out Vector3 ground)
    {
        return TryGround(point, out ground, out _);
    }

    private bool TryGround(Vector3 point, out Vector3 ground, out Vector3 normal)
    {
        ground = Vector3.zero;
        normal = Vector3.up;
        if (spawnSurface == null || !spawnSurface.enabled || spawnSurface.isTrigger)
            return false;
        Bounds bounds = spawnSurface.bounds;
        var ray = new Ray(new Vector3(point.x, bounds.max.y + 10f, point.z), Vector3.down);
        if (!spawnSurface.Raycast(ray, out RaycastHit hit, bounds.size.y + 20f)
            || Vector3.Dot(hit.normal, Vector3.up) < (hillyEpisode
                ? Mathf.Cos(maximumTerrainSlope * Mathf.Deg2Rad) : 0.9994f))
            return false;
        ground = hit.point;
        normal = hit.normal;
        return true;
    }

    private bool IsSafePose(Vector3 ground, Quaternion rotation, float rootHeight, bool ignoreObstacles = false)
    {
        // Check the whole wheel/chassis footprint, not just the center ray.
        Vector3 size = Vector3.Scale(chassisCollider.size, transform.lossyScale);
        Vector3 center = Vector3.Scale(chassisCollider.center, transform.lossyScale);
        for (int x = -1; x <= 1; x += 2)
            for (int z = -1; z <= 1; z += 2)
            {
                Vector3 corner = ground + rotation * new Vector3(center.x + x * size.x * 0.5f, 0f, center.z + z * size.z * 0.5f);
                float expectedHeight = hillyEpisode ? corner.y : ground.y;
                if (!TryGround(corner, out Vector3 support)
                    || Mathf.Abs(support.y - expectedHeight) > (hillyEpisode ? 0.12f : 0.05f))
                    return false;
            }

        // Include low obstacles around the wheels; ignore the floor and this rover.
        float top = rootHeight + center.y + size.y * 0.5f;
        Vector3 half = new Vector3(size.x * 0.5f + 0.1f, (top - 0.05f) * 0.5f, size.z * 0.5f + 0.1f);
        Vector3 volumeCenter = ground + rotation * new Vector3(center.x, 0f, center.z)
            + Vector3.up * (0.05f + half.y);
        int count = Physics.OverlapBoxNonAlloc(volumeCenter, half, spawnOverlaps, rotation, ~0, QueryTriggerInteraction.Ignore);
        if (count == spawnOverlaps.Length)
            return false;
        for (int i = 0; i < count; i++)
        {
            Collider other = spawnOverlaps[i];
            if (ignoreObstacles && obstacleField != null && obstacleField.IsObstacle(other)) continue;
            if (other != spawnSurface && other.attachedRigidbody != rb
                && !other.transform.IsChildOf(meetingPoint))
                return false;
        }
        return true;
    }

    private bool TrySupportedPose(Vector3 ground, Quaternion rotation, float rootHeight, out Vector3 position)
    {
        position = ground + Vector3.up * rootHeight;
        float lift = 0f;
        // Match the resting wheel plane to the slope without burying any tire.
        foreach (Transform wheelTransform in carControl.wheels)
        {
            WheelCollider wheel = wheelTransform.GetComponent<WheelCollider>();
            Vector3 local = transform.InverseTransformPoint(wheelTransform.TransformPoint(wheel.center));
            local = Vector3.Scale(local, transform.lossyScale);
            local.y -= wheel.radius + wheel.suspensionDistance * (1f - wheel.suspensionSpring.targetPosition);
            Vector3 tire = position + rotation * local;
            if (!TryGround(tire, out Vector3 support))
                return false;
            lift = Mathf.Max(lift, support.y + spawnClearance - tire.y);
        }
        // Excessive curvature would leave some wheels unsupported.
        if (lift > 0.12f)
            return false;
        position.y += lift;
        foreach (Transform wheelTransform in carControl.wheels)
        {
            WheelCollider wheel = wheelTransform.GetComponent<WheelCollider>();
            Vector3 local = Vector3.Scale(transform.InverseTransformPoint(
                wheelTransform.TransformPoint(wheel.center)), transform.lossyScale);
            local.y -= wheel.radius + wheel.suspensionDistance * (1f - wheel.suspensionSpring.targetPosition);
            Vector3 tire = position + rotation * local;
            if (!TryGround(tire, out Vector3 support) || tire.y - support.y > 0.12f)
                return false;
        }
        return true;
    }

    private bool TryChooseSpawn(out Vector3 position, out Quaternion rotation, out Vector3 target)
    {
        position = startPosition;
        rotation = startRotation;
        target = startTargetPosition;
        if (spawnSurface == null || chassisCollider == null || carControl.wheels.Length == 0
            || spawnAreaSize.x <= 0f || spawnAreaSize.y <= 0f)
            return false;

        // Use the wheel's resting suspension pose, with a small clearance above it.
        float rootHeight = float.MinValue;
        foreach (Transform wheelTransform in carControl.wheels)
        {
            WheelCollider wheel = wheelTransform.GetComponent<WheelCollider>();
            float centerHeight = transform.InverseTransformPoint(wheel.transform.TransformPoint(wheel.center)).y
                * transform.lossyScale.y;
            rootHeight = Mathf.Max(rootHeight, wheel.radius + wheel.suspensionDistance
                * (1f - wheel.suspensionSpring.targetPosition) - centerHeight);
        }
        rootHeight += spawnClearance;

        Bounds bounds = spawnSurface.bounds;
        float minX = Mathf.Max(startPosition.x - spawnAreaSize.x * 0.5f, bounds.min.x + spawnBoundaryMargin);
        float maxX = Mathf.Min(startPosition.x + spawnAreaSize.x * 0.5f, bounds.max.x - spawnBoundaryMargin);
        float minZ = Mathf.Max(startPosition.z - spawnAreaSize.y * 0.5f, bounds.min.z + spawnBoundaryMargin);
        float maxZ = Mathf.Min(startPosition.z + spawnAreaSize.y * 0.5f, bounds.max.z - spawnBoundaryMargin);
        if (minX >= maxX || minZ >= maxZ)
            return false;

        float minimum = Mathf.Max(minTargetDistance, targetDistance * 2f + 5f);
        float maximum = Mathf.Max(minimum, maxTargetDistance);
        float easyMinimum = Mathf.Min(maximum, Mathf.Max(minimum, 45f));
        minimum = Mathf.Lerp(easyMinimum, minimum, episodeDifficulty);
        float maxBearing = Mathf.Lerp(10f, 180f, episodeDifficulty);
        float markerOffset = startTargetPosition.y - bounds.min.y;
        if (TryGround(startTargetPosition, out Vector3 originalGround))
            markerOffset = startTargetPosition.y - originalGround.y;
        if (hillyEpisode)
            markerOffset = targetGroundOffset;

        for (int attempt = 0; attempt < 128; attempt++)
        {
            Vector3 candidate = new Vector3(Sample(minX, maxX), 0f, Sample(minZ, maxZ));
            Quaternion yaw = Quaternion.Euler(0f, Sample(-180f, 180f), 0f);
            float distance = Mathf.Sqrt(Sample(minimum * minimum, maximum * maximum));
            float bearingAngle = Sample(-maxBearing, maxBearing);
            Quaternion bearing = yaw * Quaternion.Euler(0f, bearingAngle, 0f);
            // Include blocked direct routes; the policy must learn the detour.
            if (FinalEnvironment && requireBlockedStart && obstacleField != null && obstacleField.Count > 0 && attempt < 96)
            {
                candidate = obstacleField.Anchor(spawnRandom.Next(obstacleField.Count)) - bearing * Vector3.forward * distance * 0.5f;
                if (candidate.x < minX || candidate.x > maxX || candidate.z < minZ || candidate.z > maxZ) continue;
            }
            Vector3 goal = candidate + bearing * Vector3.forward * distance;
            if (goal.x < minX || goal.x > maxX || goal.z < minZ || goal.z > maxZ)
                continue;
            if (!TryGround(candidate, out Vector3 roverGround, out Vector3 roverNormal)
                || !TryGround(goal, out Vector3 goalGround, out Vector3 goalNormal))
                continue;
            if (FinalEnvironment && currentLevel >= 4 && (!obstacleField.IsClear(roverGround, 6f)
                || !obstacleField.IsClear(goalGround, targetDistance + 4f)
                || (requireBlockedStart && !obstacleField.BlocksLine(roverGround, goalGround)))) continue;
            Quaternion roverRotation = hillyEpisode ? Quaternion.FromToRotation(Vector3.up, roverNormal) * yaw : yaw;
            Quaternion goalRotation = hillyEpisode ? Quaternion.FromToRotation(Vector3.up, goalNormal) * bearing : bearing;
            if (!IsSafePose(roverGround, roverRotation, rootHeight) || !IsSafePose(goalGround, goalRotation, rootHeight)
                || (hillyEpisode && Vector3.Angle(goalNormal, Vector3.up) > 6f))
                continue;

            bool clearPath = true;
            int samples = Mathf.CeilToInt(distance / 8f);
            for (int i = 1; i < samples; i++)
            {
                Vector3 point = Vector3.Lerp(roverGround, goalGround, (float)i / samples);
                if (!TryGround(point, out Vector3 support, out Vector3 pathNormal)
                    || (!hillyEpisode && Mathf.Abs(support.y - roverGround.y) > 0.05f)
                    || !IsSafePose(support, hillyEpisode
                        ? Quaternion.FromToRotation(Vector3.up, pathNormal) * bearing : bearing, rootHeight, FinalEnvironment && currentLevel >= 4))
                {
                    clearPath = false;
                    break;
                }
            }
            if (!clearPath)
                continue;

            position = roverGround + Vector3.up * rootHeight;
            if (hillyEpisode && !TrySupportedPose(roverGround, roverRotation, rootHeight, out position))
                continue;
            rotation = roverRotation;
            target = goalGround + Vector3.up * markerOffset;
            return true;
        }
        return false;
    }

    // Give the AI information about the target and car movement
    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 targetDirection = meetingPoint.position - rb.position;
        targetDirection.y = 0f;
        Quaternion localRotation = Quaternion.Inverse(rb.rotation);
        float speedScale = Mathf.Max(1f, carControl.maxAISpeed);

        // Eleven bounded observations, all relative to the vehicle's physics pose.
        sensor.AddObservation(localRotation * targetDirection.normalized); // 3
        sensor.AddObservation(Vector3.ClampMagnitude(localRotation * rb.linearVelocity / speedScale, 1f)); // 3
        sensor.AddObservation(carControl.CurrentSteering); // 1
        sensor.AddObservation(carControl.CurrentThrottle); // 1
        sensor.AddObservation(Mathf.Clamp((localRotation * rb.angularVelocity).y / 2f, -1f, 1f)); // 1
        float distance = targetDirection.magnitude;
        sensor.AddObservation(distance / (distance + distanceScale)); // 1
        sensor.AddObservation(Mathf.Clamp01(stoppedTime / stopDuration)); // 1
        // Empty space is zero, so migrated Level 3 weights retain their original response.
        if (obstacleObservations)
            foreach (float angle in SensorAngles)
                sensor.AddObservation(obstacleField == null ? 0f : 1f - obstacleField.ReadDistance(
                    rb.position + Vector3.up * 1.5f,
                    Quaternion.Euler(0f, rb.rotation.eulerAngles.y + angle, 0f) * Vector3.forward, sensorRange,
                    Mathf.Abs(chassisCollider.size.x * transform.lossyScale.x) * 0.5f + 0.25f));
    }

    // Receive steering and throttle actions from the AI
    public override void OnActionReceived(ActionBuffers actions)
    {
        float steering = actions.ContinuousActions[0];
        float throttle = actions.ContinuousActions[1];

        if (episodeActive)
            carControl.SetAIControls(steering, throttle);
    }

    private float GetDesiredSpeed(float distance)
    {
        // Aim inside the success radius and slow down before entering it.
        float remaining = Mathf.Max(0f, distance - targetDistance * 0.6f);
        return Mathf.Min(cruiseSpeed, Mathf.Sqrt(2f * brakingDeceleration * remaining));
    }

    private void FixedUpdate()
    {
        if (!episodeActive)
            return;

        if (FinalEnvironment && obstacleContact)
        {
            FinishEpisode(false, -2f, "ObstacleCollision");
            return;
        }
        float dt = Time.fixedDeltaTime;
        elapsedTime += dt;
        if ((hillyEpisode || FinalEnvironment) && spawnSurface != null)
        {
            Bounds bounds = spawnSurface.bounds;
            Vector3 p = rb.position;
            if (p.x < bounds.min.x || p.x > bounds.max.x || p.z < bounds.min.z
                || p.z > bounds.max.z || p.y < bounds.min.y - 5f)
            {
                FinishEpisode(false, -5f, "OutOfBounds");
                return;
            }
        }
        float distance = GetTargetDistance();
        Vector3 direction = meetingPoint.position - rb.position;
        direction.y = 0f;
        Vector3 planarVelocity = Vector3.ProjectOnPlane(rb.linearVelocity, Vector3.up);
        float closingSpeed = Vector3.Dot(planarVelocity, direction.normalized);

        // Signed progress cancels on a round trip; there is no reward for waiting.
        float stopRadius = targetDistance * 0.6f;
        float progress = Mathf.Max(0f, previousDistance - stopRadius) - Mathf.Max(0f, distance - stopRadius);
        AddReward(progress * 0.1f);
        previousDistance = distance;
        bestDistance = Mathf.Min(bestDistance, distance);

        // Penalize overspeed, not tentative movement during early exploration.
        float overspeed = Mathf.Clamp((closingSpeed - GetDesiredSpeed(distance)) / cruiseSpeed, 0f, 2f);
        AddReward(-(0.01f + overspeed * overspeed) * dt);

        Vector3 forward = rb.rotation * Vector3.forward;
        float forwardSpeed = Vector3.Dot(planarVelocity, forward);
        if (forwardSpeed < -0.1f && Vector3.Dot(direction.normalized, forward) > 0.25f)
            AddReward(-0.02f * Mathf.Clamp01(-forwardSpeed / carControl.maxReverseSpeed) * dt);

        // Allow bumps and short jumps, but end an unrecoverable episode.
        flippedTime = Vector3.Dot(rb.rotation * Vector3.up, Vector3.up) < 0.5f ? flippedTime + dt : 0f;
        airborneTime = carControl.IsGrounded ? 0f : airborneTime + dt;
        if (flippedTime >= 0.5f || airborneTime >= 5f)
        {
            FinishEpisode(false, -5f, "Unrecoverable");
            return;
        }

        bool stoppedAtTarget = distance < targetDistance && rb.linearVelocity.magnitude < stopSpeed
            && rb.angularVelocity.magnitude < 0.2f && carControl.IsGrounded;
        stoppedTime = stoppedAtTarget ? stoppedTime + dt : 0f;
        if (stoppedTime >= stopDuration)
        {
            FinishEpisode(true, 5f, "Success");
            return;
        }

        // Require meaningful net progress, while leaving time to reverse or detour.
        if (distance <= progressCheckpoint - 0.5f)
        {
            progressCheckpoint = distance;
            noProgressTime = 0f;
        }
        else
            noProgressTime += dt;

        float progressTimeout = randomizedEpisode ? level2NoProgressTimeout : noProgressTimeout;
        if (progressTimeout > 0f && noProgressTime >= progressTimeout && distance >= targetDistance)
        {
            FinishEpisode(false, -2f, "Stalled");
            return;
        }

        if (elapsedTime >= (randomizedEpisode ? level2EpisodeSeconds : maxEpisodeSeconds))
            FinishEpisode(false, -2f, "Timeout", true);
    }

    private void FinishEpisode(bool success, float terminalReward, string reason, bool interrupted = false)
    {
        episodeActive = false;
        AddReward(terminalReward);
        var stats = Academy.Instance.StatsRecorder;
        stats.Add("Rover/Success", success ? 1f : 0f);
        stats.Add("Rover/FinalDistance", GetTargetDistance());
        stats.Add("Rover/BestDistance", bestDistance);
        stats.Add("Rover/DurationSeconds", elapsedTime);
        stats.Add("Rover/Stalled", reason == "Stalled" ? 1f : 0f);
        stats.Add("Rover/Unrecoverable", reason == "Unrecoverable" ? 1f : 0f);
        stats.Add("Rover/Timeout", reason == "Timeout" ? 1f : 0f);
        if (hillyEpisode)
            stats.Add("Rover/OutOfBounds", reason == "OutOfBounds" ? 1f : 0f);
        if (randomizedEpisode)
        {
            stats.Add("Rover/SpawnDifficulty", episodeDifficulty);
            stats.Add("Rover/InitialDistance", initialDistance);
            stats.Add("Rover/InitialHeadingError", Mathf.Abs(initialHeadingError));
        }
        if (FinalEnvironment)
        {
            stats.Add("Rover/Level", currentLevel);
            stats.Add("Rover/Level" + currentLevel + "/Success", success ? 1f : 0f);
            stats.Add("Rover/ObstacleCount", obstacleField != null ? obstacleField.Count : 0);
            stats.Add("Rover/InitialPathBlocked", InitialPathBlocked ? 1f : 0f);
            if (InitialPathBlocked) stats.Add("Rover/BlockedRouteSuccess", success ? 1f : 0f);
            stats.Add("Rover/ObstacleCollision", reason == "ObstacleCollision" ? 1f : 0f);
        }
        EpisodeFinished?.Invoke(success, reason, elapsedTime, initialDistance, bestDistance, GetTargetDistance());
        if (PauseAfterEpisode)
        {
            carControl.SetAIControls(0f, 0f);
            return;
        }
        if (FinalEnvironment && success && !resetAfterSuccess && !Academy.Instance.IsCommunicatorOn)
        {
            parked = true; // Stay for the separately controlled drone in demonstrations.
            carControl.SetAIControls(0f, 0f);
            return;
        }
        if (interrupted)
            EpisodeInterrupted();
        else
            EndEpisode();
    }

    private void OnCollisionEnter(Collision collision)
    {
        // Defer reset until FixedUpdate, outside the physics callback.
        if (FinalEnvironment && episodeActive && obstacleField != null && obstacleField.IsObstacle(collision.collider))
            obstacleContact = true;
    }

    [ContextMenu("Restart Rover Episode")]
    private void RestartRoverEpisode() { if (Application.isPlaying) EndEpisode(); }

    // Select Truck in Scene view to see the same obstacle casts used by the policy.
    private void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying || !obstacleObservations || obstacleField == null || rb == null)
            return;
        Vector3 origin = rb.position + Vector3.up * 1.5f;
        float radius = Mathf.Abs(chassisCollider.size.x * transform.lossyScale.x) * 0.5f + 0.25f;
        foreach (float angle in SensorAngles)
        {
            Vector3 direction = Quaternion.Euler(0f, rb.rotation.eulerAngles.y + angle, 0f) * Vector3.forward;
            float fraction = obstacleField.ReadDistance(origin, direction, sensorRange, radius);
            Gizmos.color = fraction < 1f ? Color.red : Color.green;
            Gizmos.DrawLine(origin, origin + direction * sensorRange * fraction);
            if (fraction < 1f) Gizmos.DrawWireSphere(origin + direction * sensorRange * fraction, radius);
        }
    }

    // Test the same AI controls using the arrow keys.
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        actions[0] = Input.GetAxisRaw("Horizontal");
        actions[1] = Input.GetAxisRaw("Vertical");
    }
}
