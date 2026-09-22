using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;


[RequireComponent(typeof(Rigidbody))]
public class DroneController : Agent
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

    private bool inMP = false;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints |= RigidbodyConstraints.FreezeRotationY;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float horizontalmvmnt = actions.DiscreteActions[0]; // target tilt ApplyTiltTorque();
        
        float verticalmvmnt = actions.DiscreteActions[1]; //thrustoffset ApplyThrust();
        
        switch (horizontalmvmnt)
        {
            case 0:
            targetPitch = Mathf.Clamp(targetPitch, -maximumTiltAngle, maximumTiltAngle);
            targetRoll = Mathf.Clamp(targetRoll, -maximumTiltAngle, maximumTiltAngle);
            ApplyTiltTorque();
            break;

            case 1:
            targetPitch += tiltChangeRate * Time.fixedDeltaTime;
            ApplyTiltTorque();
            break;
            
            case 2:
            targetPitch -= tiltChangeRate * Time.fixedDeltaTime;
            ApplyTiltTorque();
            break;
            
            case 3:
            targetRoll -= tiltChangeRate * Time.fixedDeltaTime;
            ApplyTiltTorque();
            break;
            
            case 4:
            targetRoll += tiltChangeRate * Time.fixedDeltaTime;
            ApplyTiltTorque();
            break;

        }
        
        if (horizontalmvmnt == 1)
        {
            thrustOffset += thrustChangeRate * Time.fixedDeltaTime;
        }
        else if (horizontalmvmnt == 2)
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

        ApplyThrust();
    }

    private void OnllisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("DroneGoal"))
        {
            AddReward(1);
            Debug.Log("Drone landed successfully");
            EndEpisode();
        }else if (collision.gameObject.CompareTag("Building"))
        {
            AddReward(-1);
            Debug.Log("Drone crashed into building, ending episide");
            EndEpisode();
        }
    }

    private void TriggerEnter(Collider other)
    {
        if(other.gameObject.tag == "MeetingPoint")
        {
            if(!inMP){AddReward(0.5f);}
            inMP = true;

        }

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        UpdateThrustOffset();  //Här keyboard input
        UpdateTargetTilt();     //Här o
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
