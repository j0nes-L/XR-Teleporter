using Oculus.Interaction.Locomotion;
using UnityEngine;
using UnityEngine.Events;

public class XRTeleporterSnapPoint : MonoBehaviour
{
    [Tooltip("If true, the player will be rotated to match the snap point's rotation upon teleporting.")]
    [SerializeField] private bool useSnapPointRotation;
    [Tooltip("If true, the player will not teleport to this point and only the onTeleport-Event will be invoked")]
    public bool isTrigger;

    [Tooltip("Event invoked when the player teleports to this snap point")]
    public UnityEvent onTeleport;
    [Tooltip("Event invoked when the snap point becomes active")]
    public UnityEvent onSnap;
    [Tooltip("Event invoked when the snap point becomes inactive")]
    public UnityEvent onUnsnap;

    [HideInInspector] public bool isSnappable;  // True if the snap point is in range and visible
    [HideInInspector] public bool isVisible;    // True if the snap point is within the camera's view

    private Transform arrow;                    // Reference to the arrow child object  
    private XRTeleporter _xrTeleporter;         // Reference to the XRTeleporter component
    private Camera _mainCamera;                 // Reference to the main camera
    private Transform _avatar;               // Reference to the player's avatar (OVR Camera Rig)

    private bool _prevIsSnappable;              // Previous state of isSnappable
    private float _angleDifference;             // Angle difference between hand forward and direction to snap point

    private void Start()
    {
        _xrTeleporter = FindFirstObjectByType<XRTeleporter>();
        _mainCamera = Camera.main;
        _avatar = FindFirstObjectByType<OVRCameraRig>().transform;

        arrow = transform.GetChild(0);
    }

    private void Update()
    {
        arrow.gameObject.SetActive(Vector3.Distance(transform.position, _avatar.position) >= .1f);

        if (arrow.gameObject.activeSelf)
        {
            arrow.LookAt(Camera.main.transform.position);
            arrow.rotation = Quaternion.Euler(0, arrow.rotation.eulerAngles.y, 0);
        }

        CheckForSnapping();
    }

    /// <summary>
    /// Updates the snapping state of the current object and manages its presence in the list of available snap points.
    /// </summary>
    /// <remarks>This method determines whether the object is snappable by evaluating its range and updates
    /// the list of available  snap points accordingly. If the object becomes snappable, it is added to the list;
    /// otherwise, it is removed,  and any active snapping is cleared.</remarks>
    void CheckForSnapping()
    {
        isSnappable = CheckRange();

        var list = _xrTeleporter.availableSnapPoints;

        if (isSnappable && !list.Contains(this)) list.Add(this);
        else if (_prevIsSnappable)
        {
            if (list.Contains(this)) list.Remove(this);
            _xrTeleporter.Unsnap();
        }

        _prevIsSnappable = isSnappable;
    }

    /// <summary>
    /// Determines whether the object is within the valid range and angle for interaction.
    /// </summary>
    /// <remarks>This method checks if the object is visible on the screen and within the specified angle and
    /// distance constraints relative to the avatar and teleportation parameters. The visibility is determined based on
    /// the object's position in screen space, while the angle and distance checks are based on the teleportation
    /// system's configuration.</remarks>
    /// <returns><see langword="true"/> if the object is visible on the screen and satisfies the angle and distance constraints;
    /// otherwise, <see langword="false"/>.</returns>
    private bool CheckRange()
    {
        if (_mainCamera)
        {
            var screenPos = _mainCamera.WorldToScreenPoint(transform.position);
            isVisible = screenPos.x is >= 0 and <= float.MaxValue &&
            screenPos.y is >= 0 and <= float.MaxValue &&
            screenPos.x <= Screen.width && screenPos.y <= Screen.height;

            _angleDifference = CalculateYAngleDifference(_xrTeleporter.calculatedHandForward);
        }

        var avatarFlatPos = new Vector3(_avatar.transform.position.x, transform.position.y, _avatar.transform.position.z);
        var flatDistance = Vector3.Distance(transform.position, avatarFlatPos);

        bool inAngle = (_angleDifference >= -_xrTeleporter.snapAngle && _angleDifference <= _xrTeleporter.snapAngle)
        && Vector3.Distance(_xrTeleporter.teleportCheckPoint, transform.position) <= _xrTeleporter.snapDistance
        && flatDistance <= (_xrTeleporter.maxPointingRange + 1)
        && flatDistance >= _xrTeleporter.minPointingRange;

        return isVisible && inAngle;
    }

    /// <summary>
    /// Calculates the signed angle difference on the Y-axis between the given forward vector  and the direction from
    /// the hand position to the snap position.
    /// </summary>
    /// <param name="forwardVector">The forward direction vector to compare against, typically representing the hand's forward direction.</param>
    /// <returns>The signed angle difference in degrees, where positive values indicate a clockwise rotation  and negative values
    /// indicate a counterclockwise rotation. Returns 0 if the main camera is not available.</returns>
    private float CalculateYAngleDifference(Vector3 forwardVector)
    {
        if (!_mainCamera) return 0f;

        Vector3 handPos = _xrTeleporter.activeOriginTransform.position;
        Vector3 snapPos = transform.position;

        handPos.y = _avatar.transform.position.y;
        snapPos.y = _avatar.transform.position.y;

        Vector3 toPoint = (snapPos - handPos).normalized; 
        toPoint.y = 0;

        Vector3 flatHandForward = forwardVector; 
        flatHandForward.y = 0;

        toPoint.Normalize();
        flatHandForward.Normalize();

        float signedAngle = Vector3.SignedAngle(flatHandForward, toPoint, Vector3.up);
        float dot = Vector3.Dot(flatHandForward, toPoint);
        if (dot < 0) signedAngle = signedAngle > 0 ? 180 - signedAngle : -180 - signedAngle;
        return signedAngle;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(transform.position, .05f);
    }
}