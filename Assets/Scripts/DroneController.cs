using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    public float thrustChangeRate = 10.0f;
    public float maximumThrustOffset = 20.0f;
    public float thrustReturnRate = 10.0f;
    public float tiltChangeRate = 30.0f;
    public float tiltStabilizationTorque = 20.0f;
    public float angularDamping = 5.0f;
    public float maximumTiltAngle = 40.0f;
    public float minimumCosine = 0.1f;

    private Rigidbody rb;
    private float thrustOffset;
    private float targetPitch;
    private float targetRoll;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints |= RigidbodyConstraints.FreezeRotationY;
    }

    private void FixedUpdate()
    {
        UpdateThrustOffset();
        UpdateTargetTilt();
        ApplyThrust();
        ApplyTiltTorque();
        LimitTiltAngle();
    }

    private void UpdateThrustOffset()
    {
        if (Input.GetKey(KeyCode.R))
        {
            thrustOffset += thrustChangeRate * Time.fixedDeltaTime;
        }
        else if (Input.GetKey(KeyCode.F))
        {
            thrustOffset -= thrustChangeRate * Time.fixedDeltaTime;
        }
        else
        {
            thrustOffset = Mathf.MoveTowards(
                thrustOffset,
                0.0f,
                thrustReturnRate * Time.fixedDeltaTime);
        }

        thrustOffset = Mathf.Clamp(
            thrustOffset,
            -maximumThrustOffset,
            maximumThrustOffset);
    }

    private void ApplyThrust()
    {
        float cosine = Vector3.Dot(transform.up.normalized, Vector3.up);
        cosine = Mathf.Max(cosine, minimumCosine);

        float hoverThrust = (rb.mass * Physics.gravity.magnitude) / cosine;
        float thrustForce = hoverThrust + thrustOffset;

        rb.AddForce(transform.up * thrustForce, ForceMode.Force);
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

    private void UpdateTargetTilt()
    {
        if (Input.GetKey(KeyCode.W))
        {
            targetPitch += tiltChangeRate * Time.fixedDeltaTime;
        }
        if (Input.GetKey(KeyCode.S))
        {
            targetPitch -= tiltChangeRate * Time.fixedDeltaTime;
        }
        if (Input.GetKey(KeyCode.D))
        {
            targetRoll -= tiltChangeRate * Time.fixedDeltaTime;
        }
        if (Input.GetKey(KeyCode.A))
        {
            targetRoll += tiltChangeRate * Time.fixedDeltaTime;
        }

        targetPitch = Mathf.Clamp(targetPitch, -maximumTiltAngle, maximumTiltAngle);
        targetRoll = Mathf.Clamp(targetRoll, -maximumTiltAngle, maximumTiltAngle);
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
}
