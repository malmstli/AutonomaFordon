using UnityEngine;
using UnityEngine.InputSystem;


public class CleanerController : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float rotationSpeed = 180f;
 
    private Vector3 startingPosition;
    public GameObject Trash;
    private Quaternion startingRotation;

    private void Awake()
    {
        startingPosition = transform.position;
        startingRotation = transform.rotation;
    }

    public void Reset()
    {
        transform.SetPositionAndRotation(startingPosition, startingRotation);
        Trash.GetComponent<TrashRespawn>().Respawn();

        Rigidbody body = GetComponent<Rigidbody>();
        if (body != null)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }
    }

    public void CollectTrash()
    {
        Reset();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("wall"))
        {
            Reset();
        }
    }


    public void Update()
    {
        if (Keyboard.current == null)
        {
            return;
        }

        if (Keyboard.current.upArrowKey.isPressed)
        {
            transform.Translate(
                Vector3.forward * moveSpeed * Time.deltaTime,
                Space.Self
            );
        }

        if (Keyboard.current.leftArrowKey.isPressed)
        {
            transform.Rotate(
                Vector3.up * -rotationSpeed * Time.deltaTime,
                Space.Self
            );
        }
        else if (Keyboard.current.rightArrowKey.isPressed)
        {
            transform.Rotate(
                Vector3.up * rotationSpeed * Time.deltaTime,
                Space.Self
            );
        }
    }
}
