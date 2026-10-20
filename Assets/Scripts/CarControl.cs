using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarControl : MonoBehaviour
{
    public float enginePower = 2000.0f; // Engine power
    // Driving limits in metres per second, also used by manual controls.
    [Min(0.1f)]
    public float maxAISpeed = 12f;
    [Min(0.1f)]
    public float maxReverseSpeed = 3f;
    [Min(0.1f)]
    public float maxAcceleration = 3f;
    [Min(0.1f)]
    public float throttleSmoothness = 1.5f;
    // Braking force when the AI releases the throttle
    [Min(0f)]
    public float aiBrakeForce = 1500f;
    [Range(0.1f, 1f)]
    public float suspensionDampingRatio = 0.9f;
    public float turnSpeed = 25.0f; // Maximum turn speed
    public float turnSmoothness = 5.0f; // Turn smoothness
    public Transform[] wheels; // Array of Wheel Collider components
    public Transform[] wheelMeshes; // Array of wheel meshes
    public Transform centerOfMass; // Center of Mass point
    public GameObject steeringWheel;

    // AI control settings
    public bool useAI = false;
    private float aiSteering = 0f;
    private float aiThrottle = 0f;

    private Rigidbody rb; // Car's Rigidbody component
    private WheelCollider[] wheelColliders;
    private float currentTurnAngle = 0.0f;
    private float currentThrottle = 0f;
    private const float ThrottleDeadzone = 0.01f;
    private const float DirectionChangeSpeed = 0.3f;

    // Expose actuator state so the policy can observe input smoothing.
    public float CurrentSteering => currentTurnAngle / Mathf.Max(1f, turnSpeed);
    public float CurrentThrottle => currentThrottle;
    public bool IsGrounded
    {
        get
        {
            if (wheelColliders == null)
                return false;
            foreach (WheelCollider wheel in wheelColliders)
                if (wheel.isGrounded)
                    return true;
            return false;
        }
    }

    void Awake()
    {
        InitializePhysics();
    }

    private void InitializePhysics()
    {
        // The agent can reset before this component's Awake runs.
        if (rb != null)
            return;

        rb = GetComponent<Rigidbody>();
        if (centerOfMass != null)
            rb.centerOfMass = Quaternion.Inverse(rb.rotation) * (centerOfMass.position - rb.position);

        // Improve this vehicle's solver without changing shared physics settings.
        rb.solverIterations = Mathf.Max(rb.solverIterations, 12);
        rb.solverVelocityIterations = Mathf.Max(rb.solverVelocityIterations, 4);
        rb.angularDamping = Mathf.Max(rb.angularDamping, 0.5f);
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

        wheelColliders = new WheelCollider[wheels.Length];
        for (int i = 0; i < wheels.Length; i++)
            wheelColliders[i] = wheels[i].GetComponent<WheelCollider>();

        if (wheelColliders.Length == 0)
            return;

        // These settings apply to all wheels on the Rigidbody.
        wheelColliders[0].ResetSprungMasses();
        wheelColliders[0].ConfigureVehicleSubsteps(5f, 5, 3);
        wheelColliders[0].suspensionExpansionLimited = true;

        foreach (WheelCollider wheel in wheelColliders)
        {
            // Match damping to the supported mass; keep spring strength and travel.
            JointSpring spring = wheel.suspensionSpring;
            spring.damper = 2f * suspensionDampingRatio * Mathf.Sqrt(spring.spring * wheel.sprungMass);
            wheel.suspensionSpring = spring;
        }
    }

    // Receive steering and throttle inputs from the AI agent
    public void SetAIControls(float steering, float throttle)
    {
        aiSteering = Mathf.Clamp(steering, -1f, 1f);
        aiThrottle = Mathf.Clamp(throttle, -1f, 1f);
    }

    // Reset the steering and wheel forces between episodes
    public void ResetControls()
    {
        InitializePhysics();
        aiSteering = 0f;
        aiThrottle = 0f;
        currentTurnAngle = 0f;
        currentThrottle = 0f;

        foreach (WheelCollider wc in wheelColliders)
        {
            wc.motorTorque = 0f;
            wc.brakeTorque = 0f;
            wc.steerAngle = 0f;
            wc.rotationSpeed = 0f; // Clear wheel spin as well as chassis velocity.
        }
    }

    void FixedUpdate()
    {
        // Choose between AI control and manual keyboard control.
        float horizontalInput = 0.0f;
        float throttle = 0.0f;

        if (useAI)
        {
            horizontalInput = aiSteering;
            throttle = aiThrottle;
        }
        else
        {
            // Use arrow keys for manual control.
            if (Input.GetKey(KeyCode.LeftArrow))
                horizontalInput = -1.0f;
            else if (Input.GetKey(KeyCode.RightArrow))
                horizontalInput = 1.0f;

            if (Input.GetKey(KeyCode.UpArrow))
                throttle = 1.0f;
            else if (Input.GetKey(KeyCode.DownArrow))
                throttle = -1.0f;
        }

        float forwardSpeed = Vector3.Dot(rb.linearVelocity, rb.rotation * Vector3.forward);
        float speed = Mathf.Abs(forwardSpeed);
        float speedLimit = forwardSpeed >= 0f ? maxAISpeed : maxReverseSpeed;
        float steeringScale = Mathf.Lerp(1f, 0.35f, Mathf.Clamp01(speed / Mathf.Max(0.1f, maxAISpeed)));

        // Keep full low-speed steering, but reduce sharp turns at speed.
        float targetTurnAngle = horizontalInput * turnSpeed * steeringScale;
        currentTurnAngle = Mathf.Lerp(currentTurnAngle, targetTurnAngle,
            1f - Mathf.Exp(-turnSmoothness * Time.fixedDeltaTime));

        if (steeringWheel != null)
            steeringWheel.transform.localEulerAngles = new Vector3(-64, 0, currentTurnAngle * 3);

        // Keep a small neutral band without discarding useful PPO actions.
        float requestedThrottle = Mathf.Abs(throttle) <= ThrottleDeadzone ? 0f : throttle;
        bool changingDirection = requestedThrottle * forwardSpeed < 0f && speed > DirectionChangeSpeed;
        float brake = 0f;

        if (requestedThrottle == 0f || changingDirection)
        {
            currentThrottle = 0f;
            // Opposite input gives proportional braking before reversing.
            if (changingDirection)
                brake = aiBrakeForce * Mathf.Abs(requestedThrottle);
            else if (useAI)
                brake = aiBrakeForce;
        }
        else
        {
            if (currentThrottle * requestedThrottle < 0f)
                currentThrottle = 0f;
            currentThrottle = Mathf.MoveTowards(currentThrottle, requestedThrottle,
                throttleSmoothness * Time.fixedDeltaTime);
        }

        // Taper drive torque near each limit and brake overspeed, including on slopes.
        float driveLimit = currentThrottle >= 0f ? maxAISpeed : maxReverseSpeed;
        float driveSpeed = Mathf.Max(0f, forwardSpeed * Mathf.Sign(currentThrottle));
        float torqueScale = Mathf.Clamp01((driveLimit - driveSpeed) / 1f);
        if (speed > speedLimit)
            brake = Mathf.Max(brake, aiBrakeForce * Mathf.Clamp01((speed - speedLimit) / 1f));

        // Wheels rotation and steering
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            WheelCollider wheelCollider = wheelColliders[i];
            if (i < 2) // First two wheels steer
            {
                wheelCollider.steerAngle = currentTurnAngle;
            }
            else
            {
                wheelCollider.steerAngle = 0f; // Other two wheels remain straight
            }
            // Limit total drive force while retaining enginePower as the torque ceiling.
            float torqueLimit = Mathf.Min(enginePower,
                rb.mass * maxAcceleration * wheelCollider.radius / wheelColliders.Length);
            wheelCollider.brakeTorque = brake;
            wheelCollider.motorTorque = brake > 0f ? 0f : currentThrottle * torqueLimit * torqueScale;

            // Rotate wheel meshes
            if (wheelMeshes == null || i >= wheelMeshes.Length || wheelMeshes[i] == null)
                continue;
            Quaternion wheelRotation;
            Vector3 wheelPosition;
            wheelCollider.GetWorldPose(out wheelPosition, out wheelRotation);
            wheelMeshes[i].position = wheelPosition;
            wheelMeshes[i].rotation = wheelRotation;
        }

    }
}
