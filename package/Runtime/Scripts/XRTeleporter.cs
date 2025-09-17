using Oculus.Interaction;
using Oculus.Interaction.HandGrab;
using Oculus.Interaction.Input;
using Oculus.Interaction.PoseDetection;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class XRTeleporter : MonoBehaviour
{
    [Header("Basic Constraints:")]
    [Tooltip("Master switch for the whole system. If false, the teleporter UI and input are suppressed.")]
    public bool allowTeleportation = true;
    [Tooltip("How targets are chosen: Free = raycasted surface; SnapPoints = snaps to XRTeleporterSnapPoint objects.")]
    public TeleportMode teleportMode = TeleportMode.Free;
    [Tooltip("Bitmask of optional behaviors. Toggle features via the TeleportFeature flags (e.g., RotationWithJoysticks).")]
    public TeleportFeature features;
    [Tooltip("If true, enables debug logging to the console.")]
    [SerializeField] private bool debugMode;

    [Header("General Properties:")]
    [Tooltip("Controller button used with controllers: hold to aim, release to teleport (if allowed).")]
    [SerializeField] private OVRInput.Button teleportButton = OVRInput.Button.One;
    [Tooltip("Maximum distance (in meters) for hit tests and target placement.")]
    [Range(1, 50)] public float maxPointingRange = 12;
    [Tooltip("Minimum distance from the player at which the curve/target will be considered; prevents very close teleports.")]
    [Range(0, 5)] public float minPointingRange = 1;
    [Tooltip("Vertical offset (in meters) applied to valid targets to avoid z-fighting with the ground.")]
    [SerializeField, Range(0, .2f)] private float surfaceOffset = .05f;
    [Tooltip("Max allowed ground slope (degrees). Surfaces steeper than this are rejected as teleport targets.")]
    [SerializeField, Range(0, 90)] private float maxAcceptableSurfaceAngle = 45;
    [Tooltip("Resolution of the upward arc check when facing a wall (number of rays in the vertical sweep). Higher = smoother but costlier.")]
    [SerializeField, Range(90, 270)] private float heightCheckRayCount = 180;
    [Tooltip("Required headroom (in meters). If an upward ray from the target hits something within this distance, the target is invalid.")]
    [SerializeField] private float verticalClearance = 3;
    [Tooltip("Lerp speed for horizontal movement of the target (XZ). Higher values feel snappier.")]
    [SerializeField] private float horizontalSmoothingFactor = 9;
    [Tooltip("Lerp speed for vertical movement of the target (Y). Higher values feel snappier.")]
    [SerializeField] private float verticalSmoothingFactor = 17;
    [Tooltip("Cooldown after (hand-tracking) teleport before the teleporter can activate again.")]
    [SerializeField] private float handTrackingTeleportCooldownTimer = .5f;
    [Tooltip("Hand orientation threshold (degrees) considered as 'aiming' when using hand tracking.")]
    [SerializeField][Range(20, 90)] private float handTrackingTeleportHandAngle = 30;
    [Tooltip("Max distance (in meters) to consider a snap point nearby (used by snap-based teleport logic).")]
    public float snapDistance = 4;
    [Tooltip("Angular tolerance (degrees) when checking if the current snap point lies within the aim direction.")]
    public float snapAngle = 30;
    [Tooltip("Degrees rotated per joystick step when RotationWithJoysticks is enabled.")]
    [SerializeField, Range(30, 90)] private float rotationStep = 45;
    [Tooltip("Layers to ignore for all physics checks (raycasts).")]
    [SerializeField] private LayerMask layerMaskToIgnore;

    [Header("Poses (Meta):")]
    [Tooltip("Hand-tracking pose that arms the teleporter (aim pose).")]
    [SerializeField] private ShapeRecognizer teleportAimPose;
    [Tooltip("Hand-tracking pose that confirms and triggers the teleport.")]
    [SerializeField] private ShapeRecognizer teleportConfirmPose;

    [Header("Curve & Target Properties:")]
    [Tooltip("Number of segments used to build the arc mesh. More segments = smoother curve.")]
    [SerializeField, Range(10, 50)] private int curveSegments = 20;
    [Tooltip("Visual width of the arc mesh (in meters).")]
    [SerializeField, Range(.01f, 1)] private float curveWidth = 0.75f;
    [Tooltip("Maximum arc height (in meters) at 90° aim; lowers as the aim flattens.")]
    [SerializeField, Range(1, 10)] private float maxCurveHeight = 1.25f;
    [Tooltip("Curve/target color when the teleport is valid.")]
    [SerializeField] private Color defaultColor = Color.green;
    [Tooltip("Curve/target color when the teleport is invalid (blocked, no clearance, etc.).")]
    [SerializeField] private Color notValidColor = Color.red;
    [Tooltip("Curve color when the surface is too steep (above Max Acceptable Surface Angle).")]
    [SerializeField] private Color angleTooBigColor = Color.gray;
    [Tooltip("If true, writes the outer target color to the material's emission (for a glowing target).")]
    [SerializeField] private bool useEmissiveMaterials = true;

    [HideInInspector] public List<XRTeleporterSnapPoint> availableSnapPoints = new();   // All snap points currently considered "in range" (filled externally)
    [HideInInspector] public bool teleporterActive, isValidTeleport;                    // Runtime state: teleporter UI active, and whether the current target is valid
    [HideInInspector] public Transform activeOriginTransform, teleportTarget;           // Current aiming origin (hand or controller) and the visual target marker
    [HideInInspector] public Vector3 teleportCheckPoint, calculatedHandForward;         // Candidate target position from raycasts and the derived aim direction

    private Transform avatar;                                                           // Player rig root (OVRCameraRig.transform)
    private bool handTrackingActive;                                                    // True while OVRInput active controller == Hands

    private bool facingWall, isSnapped, noVerticalClearance, useSnapPointForward;       // Aiming flags: wall in front, snapped to point, blocked headroom, align to snap forward
    private bool previousHandTracking, handTrackingTeleportOnCooldown, hasInitializedTargetPosition; // Hand-tracking state: mode change edge, cooldown lock, target seeded

    private float height, distance, lastValidHeight;                                    // Arc peak scalar, distance to target, last valid Y (for fallback)
    private float avatarRotationY;                                                      // Yaw applied to the avatar after teleport
    private float setHorizontalSmoothingFactor, setVerticalSmoothingFactor;             // Effective smoothing factors for target Lerp this frame

    private HandCtx _right, _left;                                                      // Cached contexts for right/left (transforms + interactors)
    private XRTeleporterSnapPoint activeSnapPoint;                                      // Currently selected snap point (if any)
    private MeshRenderer curveRenderer, targetRenderer;                                 // Renderers for arc mesh and target reticle
    private Vector2 defaultTextureTiling = new Vector2(1, .76f);                        // UV tiling for the arc texture in the default (valid) state.

    private Vector3 tpOrigin, targetPosition, smoothedTargetPosition, highestPoint;     // Computed origin, desired target, smoothed target, and highest probe point
    private Quaternion teleportRotation;                                                // Final rotation to apply on teleport (usually yaw only)
    private LayerMask layerMask;                                                        // Layers included in physics checks (inverse of layerMaskToIgnore)
    private Coroutine inactiveCoroutine;                                                // Handle for delayed deactivate coroutine
    private Camera _mainCamera;                                                         // Cached reference to Camera.main


    public static XRTeleporter Instance { get; private set; }

    private void OnValidate()
    {
        if (PlayerPrefs.GetInt("XRTeleporter_Initialized") == 0)
        {
            SetupAvatar();
            PlayerPrefs.SetInt("XRTeleporter_Initialized", 1);
        }
    }

    private void Awake()
    {
        if (!Instance) Instance = this;

        _right = new HandCtx
        {
            Side = Handedness.Right,
            Controller = GameObject.Find("RightControllerAnchor").transform,
            Hand = GameObject.Find("OVRRightHandVisual").transform,
            OvrController = OVRInput.Controller.RTouch,
        };
        _right.GetInteractors();

        _left = new HandCtx
        {
            Side = Handedness.Left,
            Controller = GameObject.Find("LeftControllerAnchor").transform,
            Hand = GameObject.Find("OVRLeftHandVisual").transform,
            OvrController = OVRInput.Controller.LTouch
        };
        _left.GetInteractors();
    }

    private void Start()
    {
        avatar = FindFirstObjectByType<OVRCameraRig>().transform;
        layerMask = ~layerMaskToIgnore;
        curveRenderer = GetComponent<MeshRenderer>();

        _mainCamera = Camera.main;

        teleportTarget = transform.GetChild(0);
        targetRenderer = teleportTarget.GetComponent<MeshRenderer>();
        teleportTarget.gameObject.SetActive(false);
        transform.SetParent(avatar);

        activeOriginTransform = handTrackingActive ? _right.Hand : _right.Controller;
        calculatedHandForward = CalculateHandForward();
        StartCoroutine(HandTrackingTeleportCooldown());

        if(!teleportAimPose || !teleportConfirmPose)
            Debug.LogError("Missing Shape Recognizer! You can find prebuild poses under: Packages > Meta XR Interaction SDK Essentials > Runtime > Sample > Poses > Shapes");

        if (useEmissiveMaterials) targetRenderer.materials[1].EnableKeyword("_EMISSION");
        else targetRenderer.materials[1].DisableKeyword("_EMISSION");

        SetupPoseDetection();
    }

    private void Update()
    {
        if (allowTeleportation)
        {
            handTrackingActive = OVRInput.GetActiveController() == OVRInput.Controller.Hands;

            if (!Instance.handTrackingActive) TeleporterInputController();

            if (handTrackingActive != previousHandTracking)
            {
                teleporterActive = false;
                previousHandTracking = handTrackingActive;
            }
        }
        else teleportTarget.gameObject.SetActive(false);

        if (features.HasFlag(TeleportFeature.RotationWithJoysticks))
            RotationInput();
    }

    private void LateUpdate()
    {
        if (!teleporterActive && !hasInitializedTargetPosition)
            return;

        HandleInteractors();

        CalculateTeleportPoint();
        TargetSmoothing();
        GenerateCurve();

        DrawHandDebugRays(_right.Hand, Color.blue, Color.magenta, Color.yellow, Color.green);
        DrawHandDebugRays(_left.Hand, Color.blue, Color.magenta, Color.yellow, Color.green);
    }

    #region General

    /// <summary>
    /// Activates the teleporter and sets its target position based on the specified origin.
    /// </summary>
    /// <remarks>This method initializes the teleporter's state, calculates the teleportation target position,
    /// and updates the teleporter's target transform. Ensure that the <paramref name="origin"/> parameter is not null
    /// to avoid unexpected behavior.</remarks>
    /// <param name="origin">The transform representing the origin point from which the teleporter is activated. This determines the target
    /// position for teleportation.</param>
    private void ActivateTeleporter(Transform origin)
    {
        ClearTeleporterFlags();
        activeOriginTransform = origin;
        teleporterActive = true;

        CalculateTeleportPoint();
        targetPosition = teleportCheckPoint;
        teleportTarget.position = targetPosition;
        hasInitializedTargetPosition = true;
    }

    /// <summary>
    /// Activates the teleporter using the specified hand context.
    /// </summary>
    /// <remarks>The teleporter will only activate if it is not already active and the hand's aiming is
    /// active.</remarks>
    /// <param name="hand">The context of the hand used to activate the teleporter.  The hand must have aiming active, and if the hand is
    /// not selecting, it will be used as the origin for the teleporter.</param>
    private void ActivateTeleporterWithHand(HandCtx hand)
    {
        if (teleporterActive || !hand.AimingActive) return;

        if (!hand.IsSelecting) activeOriginTransform = hand.Hand;
        ActivateTeleporter(activeOriginTransform);
    }

    /// <summary>
    /// Handles the interaction logic for hands, managing teleporter activation and deactivation based on the current
    /// state of each hand.
    /// </summary>
    /// <remarks>This method iterates through all hands and evaluates their interaction states to determine
    /// whether to activate or deactivate the teleporter. It ensures that the teleporter is only activated or
    /// deactivated under specific conditions, such as hovering, selecting, or aiming.</remarks>
    private void HandleInteractors()
    {
        foreach (var hand in EnumerateHands())
        {
            bool selecting = hand.IsHovering || hand.IsSelecting;
            bool isActiveOrigin = activeOriginTransform == hand.Hand;

            if (teleporterActive && selecting && isActiveOrigin && !hand.DeactivatedByHover)
            {
                hand.DeactivatedByHover = true;
                ClearTeleporterFlags();
            }
            else if (hand.DeactivatedByHover && !selecting && hand.AimingActive)
            {
                hand.DeactivatedByHover = false;
                ActivateTeleporterWithHand(hand);
            }
        }
    }

    /// <summary>
    /// Calculates the forward direction of the hand based on the current hand tracking state.
    /// </summary>
    /// <returns>A <see cref="Vector3"/> representing the forward direction of the hand. If hand tracking is inactive,  the
    /// method returns the forward direction of the active origin transform. Otherwise, it returns a  direction based on
    /// the active hand's orientation.</returns>
    private Vector3 CalculateHandForward()
    {
        if (!handTrackingActive) return activeOriginTransform.forward;
        return activeOriginTransform == _right.Hand ? -activeOriginTransform.right : activeOriginTransform.right;
    }

    /// <summary>
    /// Calculates the teleportation point based on the given origin transform.
    /// </summary>
    /// <param name="origin">The transform representing the origin point for the calculation.</param>
    /// <returns>A <see cref="Vector3"/> representing the calculated teleportation point.  If hand tracking is inactive, the
    /// method returns the forward direction of the origin. Otherwise, it returns a point offset from the origin's
    /// position in the calculated hand forward direction.</returns>
    private Vector3 CalculateTeleportPoint(Transform origin)
    {
        if (!handTrackingActive) return origin.forward;
        return origin.position + (calculatedHandForward * .15f);
    }

    /// <summary>
    /// Resets the teleporter's state and clears any active flags.
    /// </summary>
    /// <remarks>This method deactivates the teleporter, resets the target position to the teleport
    /// checkpoint,  and updates the teleport target's position accordingly. It ensures the teleporter is in a  default,
    /// inactive state.</remarks>
    private void ClearTeleporterFlags()
    {
        teleporterActive = hasInitializedTargetPosition = false;
        targetPosition = teleportCheckPoint;
        teleportTarget.position = targetPosition;
    }

    #endregion

    #region Input

    /// <summary>
    /// Handles input for activating and performing teleportation using the teleporter system.
    /// </summary>
    /// <remarks>This method checks the state of the teleportation button for both the left and right
    /// controllers. If the teleportation button is pressed and the respective controller is not blocked (e.g.,
    /// selecting or hovering), the teleporter is activated for that controller. If the teleportation button is released
    /// and teleportation is allowed, the teleportation action is performed.</remarks>
    private void TeleporterInputController()
    {
        bool rightBlocked = _right.IsSelecting || _right.IsHovering;
        bool leftBlocked = _left.IsSelecting || _left.IsHovering;

        if (OVRInput.Get(teleportButton, _right.OvrController) && !rightBlocked) ActivateTeleporter(_right.Controller);
        else if (OVRInput.Get(teleportButton, _left.OvrController) && !leftBlocked) ActivateTeleporter(_left.Controller);

        if (OVRInput.GetUp(teleportButton, _right.OvrController) && activeOriginTransform == _right.Controller && allowTeleportation) Teleport();
        else if (OVRInput.GetUp(teleportButton, _left.OvrController) && activeOriginTransform == _left.Controller && allowTeleportation) Teleport();
    }

    /// <summary>
    /// Handles avatar rotation based on input from the primary thumbstick of the VR controllers.
    /// </summary>
    /// <remarks>This method checks for input from the primary thumbstick on both the left and right VR
    /// controllers. Pressing the right direction rotates the avatar clockwise, while pressing the left direction
    /// rotates the avatar counterclockwise. The rotation step is determined by the <c>rotationStep</c> value.</remarks>
    private void RotationInput()
    {
        if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstickRight, _right.OvrController)) RotateAvatar(rotationStep);
        else if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstickRight, _left.OvrController)) RotateAvatar(rotationStep);

        if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstickLeft, _right.OvrController)) RotateAvatar(-rotationStep);
        else if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstickLeft, _left.OvrController)) RotateAvatar(-rotationStep);
    }

    /// <summary>
    /// Activates the teleporter aim pose for the specified hand.
    /// </summary>
    /// <remarks>This method queues the aim pose activation for the specified hand and attempts to activate
    /// the teleporter  based on the current aim state.</remarks>
    /// <param name="rightSide">A boolean value indicating which hand to activate the aim pose for.  <see langword="true"/> activates the right
    /// hand; <see langword="false"/> activates the left hand.</param>
    public void SetTeleporterAimPoseActive(bool rightSide)
    {
        var hand = GetHand(rightSide);
        hand.AimQueued = true;
        TryActivateFromAim(hand);
    }

    /// <summary>
    /// Deactivates the teleporter aim pose for the specified side.
    /// </summary>
    /// <remarks>This method stops any currently running coroutine for deactivation and starts a new coroutine
    /// to handle the deactivation process. Teleportation must be allowed for this method to take effect.</remarks>
    /// <param name="rightSide">A boolean value indicating which side to deactivate.  <see langword="true"/> deactivates the right side; <see
    /// langword="false"/> deactivates the left side.</param>
    public void SetTeleporterAimPoseInactive(bool rightSide)
    {
        if ((!allowTeleportation) || (rightSide && _left.AimingActive) || (!rightSide && _right.AimingActive)) return;
        if (inactiveCoroutine != null) StopCoroutine(inactiveCoroutine);

        inactiveCoroutine = StartCoroutine(SetInactiveCoroutine(rightSide));
    }

    /// <summary>
    /// Confirms the pose detection for teleportation and initiates the teleportation process if conditions are met.
    /// </summary>
    /// <remarks>This method checks the current state of the teleporter and the active snap point. If the
    /// teleporter is active  and a valid snap point is set, it evaluates the aiming and selection states of the left
    /// and right controllers  to determine whether to perform a teleportation action.</remarks>
    public void TeleporterConfirmPoseDetected()
    {
        if (!teleporterActive || !activeSnapPoint) return;

        bool selectingRight = _right.IsSelecting;
        bool selectingLeft = _left.IsSelecting;

        if ((_right.AimingActive && !selectingRight) || (_left.AimingActive && !selectingLeft)) Teleport();
    }

    /// <summary>
    /// Attempts to activate the teleporter based on the aiming state of the specified hand.
    /// </summary>
    /// <remarks>This method checks various conditions, such as whether the teleporter is already active, 
    /// whether the hand is in a valid state for activation, and whether the other hand is also aiming.  If the
    /// conditions are met, the teleporter is activated for the specified hand.</remarks>
    /// <param name="hand">The hand context representing the hand attempting to activate the teleporter.</param>
    private void TryActivateFromAim(HandCtx hand)
    {
        if (handTrackingTeleportOnCooldown || teleporterActive) return;

        bool otherAiming = GetOtherHand(hand).AimingActive;
        bool pinchingVisible = CheckForHandVisibility(hand.Hand);

        bool shouldReactivate = !teleporterActive && hand.AimingActive && !handTrackingTeleportOnCooldown && !hand.IsHovering && !hand.IsSelecting;

        if ((hand.AimQueued && !otherAiming && pinchingVisible) || shouldReactivate)
        {
            hand.AimQueued = false;
            hand.AimingActive = true;

            if (!hand.IsHovering)
                ActivateTeleporterWithHand(hand);
        }
    }

    /// <summary>
    /// Deactivates the aiming functionality for the specified hand after a short delay.
    /// </summary>
    /// <remarks>This coroutine waits for a brief period before checking the hover state of the specified
    /// hand.  If the hand is not hovering, it disables aiming-related functionality for that hand.  Additionally, it
    /// clears any teleporter-related flags.</remarks>
    /// <param name="rightSide">A value indicating whether to target the right hand (<see langword="true"/>) or the left hand (<see
    /// langword="false"/>).</param>
    /// <returns></returns>
    private IEnumerator SetInactiveCoroutine(bool rightSide)
    {
        yield return new WaitForSeconds(0.1f);
        var hand = rightSide ? _right : _left;

        if (!hand.IsHovering)
        {
            hand.AimQueued = false;
            hand.AimingActive = false;
        }

        ClearTeleporterFlags();
    }

    /// <summary>
    /// Determines whether the specified target is visible within the camera's viewport and within the allowed angular
    /// range.
    /// </summary>
    /// <remarks>The method checks if the target is within the camera's viewport and evaluates its orientation
    /// relative to predefined angular constraints. The visibility is determined based on the target's position in the
    /// viewport and its alignment with the forward and right vectors.</remarks>
    /// <param name="target">The transform of the target to check for visibility.</param>
    /// <returns><see langword="true"/> if the target is visible in the camera's viewport and within the specified angular
    /// constraints; otherwise, <see langword="false"/>.</returns>
    private bool CheckForHandVisibility(Transform target)
    {
        Vector3 viewportPoint = _mainCamera.WorldToViewportPoint(target.position) + CalculateHandForward() * .1f;
        bool isVisible = viewportPoint.z > 0 && viewportPoint.x is >= 0 and <= 1 && viewportPoint.y is >= 0 and <= 1;

        Vector3 forwardVector = calculatedHandForward;
        Vector3 rightVector = activeOriginTransform == _right.Controller || activeOriginTransform == _right.Hand ? -target.forward : target.forward;

        Vector3 projectedForward = new(forwardVector.x, 0, forwardVector.z);
        float angleForward = Vector3.Angle(projectedForward, forwardVector);

        float angleRight = Vector3.Angle(rightVector, Vector3.up);

        bool inAngle = angleForward <= handTrackingTeleportHandAngle && angleRight <= handTrackingTeleportHandAngle
            && angleForward >= -handTrackingTeleportHandAngle && angleRight >= -handTrackingTeleportHandAngle;

        return isVisible && inAngle;
    }

    /// <summary>
    /// Temporarily disables hand-tracking teleport functionality for a cooldown period.
    /// </summary>
    /// <remarks>This method sets the teleport functionality to a cooldown state, preventing its use for the
    /// duration specified by <c>handTrackingTeleportCooldownTimer</c>. After the cooldown period elapses, the
    /// functionality is re-enabled.</remarks>
    /// <returns></returns>
    private IEnumerator HandTrackingTeleportCooldown()
    {
        handTrackingTeleportOnCooldown = true;
        yield return new WaitForSeconds(handTrackingTeleportCooldownTimer);
        handTrackingTeleportOnCooldown = false;
    }

    #endregion

    #region Teleportation / Rotation

    /// <summary>
    /// Teleports the avatar to the designated teleport target position and adjusts the camera and rotation accordingly.
    /// </summary>
    /// <remarks>This method performs a teleportation action if the teleportation conditions are met, such as
    /// the teleporter being active  and the teleport target being valid. It also invokes any associated teleportation
    /// events, resets aiming states, and clears  teleportation-related flags. If the avatar is snapped to a snap point,
    /// it will unsnap after teleportation.</remarks>
    public void Teleport()
    {
        StartCoroutine(HandTrackingTeleportCooldown());

        if (!(isSnapped && activeSnapPoint.isTrigger))
        {
            if (isValidTeleport && teleporterActive)
            {
                avatar.position = teleportTarget.position;
                _mainCamera.transform.position = new Vector3(avatar.position.x, _mainCamera.transform.position.y, avatar.position.z);
                SetTeleportRotation();
            }
        }

        activeSnapPoint?.onTeleport?.Invoke();
        _right.AimingActive = _left.AimingActive = false;

        ClearTeleporterFlags();
        if (isSnapped) Unsnap();
    }

    /// <summary>
    /// Sets the avatar's rotation to align with the forward direction of the active snap point.
    /// </summary>
    /// <remarks>This method updates the avatar's rotation only if snapping is enabled and the snap point's
    /// forward direction is being used.</remarks>
    private void SetTeleportRotation()
    {
        if (!isSnapped || !useSnapPointForward) return;

        avatarRotationY = activeSnapPoint.transform.eulerAngles.y;
        teleportRotation = Quaternion.Euler(0, avatarRotationY, 0);
        avatar.rotation = teleportRotation;
    }

    /// <summary>
    /// Rotates the avatar around its vertical axis by the specified step value.
    /// </summary>
    /// <remarks>This method also updates the camera's position to align horizontally with the avatar's
    /// position and disables teleportation functionality by setting <see langword="teleporterActive"/> to <see
    /// langword="false"/>  and <see langword="isValidTeleport"/> to <see langword="false"/>.</remarks>
    /// <param name="step">The angle, in degrees, by which to rotate the avatar. Positive values rotate clockwise, and negative values
    /// rotate counterclockwise.</param>
    private void RotateAvatar(float step)
    {
        _mainCamera.transform.position = new Vector3(avatar.position.x, _mainCamera.transform.position.y, avatar.position.z);
        avatar.Rotate(0f, step, 0f);

        teleporterActive = false;
        isValidTeleport = false;
    }

    #endregion

    #region Aiming & Point Calculation / Helpers

    /// <summary>
    /// Calculates the teleportation point based on the current hand position, orientation, and other relevant factors.
    /// </summary>
    /// <remarks>This method determines the teleportation point by analyzing the hand's forward direction,
    /// checking for obstacles, and applying additional logic based on the teleportation mode. If a valid target is
    /// detected, it updates the  teleportation target's position and orientation. If no valid target is found, fallback
    /// handling is applied.</remarks>
    private void CalculateTeleportPoint()
    {
        calculatedHandForward = CalculateHandForward();
        tpOrigin = CalculateTeleportPoint(activeOriginTransform);

        Vector3 handAngle = handTrackingActive
            ? Vector3.ProjectOnPlane(transform.TransformDirection(calculatedHandForward), Vector3.up).normalized
            : Quaternion.Euler(0f, activeOriginTransform.eulerAngles.y, 0) * Vector3.forward;

        facingWall = Physics.Raycast(tpOrigin, handAngle, maxPointingRange * .98f, layerMask);

        distance = Vector3.Distance(teleportTarget.position, avatar.position);
        teleportTarget.rotation = Quaternion.Euler(0f, teleportTarget.rotation.eulerAngles.y, 0f);

        if (teleportMode == TeleportMode.SnapPoints && teleporterActive && hasInitializedTargetPosition)
            SnapToClosestSnapPoint();

        if (Physics.Raycast(tpOrigin, calculatedHandForward, out RaycastHit hit, maxPointingRange, layerMask))
        {
            HandleHitResult(hit);
            return;
        }

        HandleNoHit();
        teleportTarget.rotation = Quaternion.Euler(0f, teleportTarget.rotation.eulerAngles.y, 0f);
    }

    /// <summary>
    /// Processes the result of a raycast hit to determine the validity of a teleportation target.
    /// </summary>
    /// <remarks>This method evaluates the surface angle, snap point validity, and vertical clearance at the
    /// hit location  to determine whether the teleportation target is valid. It adjusts the teleportation checkpoint
    /// position  and applies appropriate visual feedback based on the evaluation results.</remarks>
    /// <param name="hit">The <see cref="RaycastHit"/> object containing information about the raycast collision.</param>
    private void HandleHitResult(RaycastHit hit)
    {
        Vector3 hitPoint = hit.point;
        float angle = Vector3.Angle(hit.normal, Vector3.up);

        if (angle >= maxAcceptableSurfaceAngle)
        {
            teleportCheckPoint = hitPoint;
            isValidTeleport = false;
            ApplyCurveAngleTooBig();

            setVerticalSmoothingFactor = verticalSmoothingFactor;
            setHorizontalSmoothingFactor = horizontalSmoothingFactor;
            SetToPosition(teleportCheckPoint);
            return;
        }

        teleportCheckPoint = new Vector3(hitPoint.x, hitPoint.y + surfaceOffset, hitPoint.z);
        SetToPosition(teleportCheckPoint);

        if (teleportMode == TeleportMode.SnapPoints)
        {
            bool validSnap = isSnapped && CurrentSnapPointInAngle();
            isValidTeleport = validSnap;

            if (validSnap) ApplyCurveValid();
            else ApplyCurveInvalid(features.HasFlag(TeleportFeature.FadeOutCurveOnNoTarget));
        }
        else
        {
            noVerticalClearance = Physics.Raycast(teleportCheckPoint, Vector3.up, verticalClearance, layerMask);
            isValidTeleport = !noVerticalClearance;

            if (isValidTeleport) ApplyCurveValid();
            else ApplyCurveInvalidSolid();
        }

        setVerticalSmoothingFactor = verticalSmoothingFactor;
        setHorizontalSmoothingFactor = horizontalSmoothingFactor;
    }

    /// <summary>
    /// Handles the behavior when no valid teleportation target is detected.
    /// </summary>
    /// <remarks>This method adjusts smoothing factors and determines the appropriate action based on the
    /// current  state of the teleporter, such as whether it is facing a wall, whether snapping to geometry is enabled, 
    /// and whether the teleporter is active. It also updates the validity of the teleportation target.</remarks>
    private void HandleNoHit()
    {
        if (facingWall && teleporterActive && !isSnapped)
        {
            setVerticalSmoothingFactor = 5;

            if (features.HasFlag(TeleportFeature.SnapToHighestEdge)) StartCoroutine(CalculateHighestPoint());
            else FadeOutCurve();

            isValidTeleport = false;
            ApplyCurveAngleTooBig();
        }
        else
        {
            setVerticalSmoothingFactor = verticalSmoothingFactor;
            MaxRangeTeleport();
        }

        setHorizontalSmoothingFactor = horizontalSmoothingFactor;
    }

    /// <summary>
    /// Smoothly interpolates the position of the teleport target towards the desired target position.
    /// </summary>
    /// <remarks>This method adjusts the teleport target's position by applying horizontal and vertical
    /// smoothing factors. It only performs the smoothing operation if the target position has been
    /// initialized.</remarks>
    private void TargetSmoothing()
    {
        if (!hasInitializedTargetPosition) return;

        smoothedTargetPosition = Vector3.Lerp(teleportTarget.position, targetPosition, Time.deltaTime * setHorizontalSmoothingFactor);
        smoothedTargetPosition.y = Mathf.Lerp(teleportTarget.position.y, targetPosition.y, Time.deltaTime * setVerticalSmoothingFactor);
        teleportTarget.position = smoothedTargetPosition;
    }

    /// <summary>
    /// Calculates the highest reachable point within a specified range and updates the position accordingly.
    /// </summary>
    /// <remarks>This method uses raycasting to determine the highest point along a series of angles projected
    /// from the origin. The calculation ignores objects in the specified layer mask and considers only objects within
    /// the maximum pointing range. The result is used to update the position to the calculated highest point.</remarks>
    /// <returns></returns>
    private IEnumerator CalculateHighestPoint()
    {
        yield return new WaitForEndOfFrame();

        LayerMask lm = ~layerMaskToIgnore;
        Vector3 projectedForward = Vector3.ProjectOnPlane(calculatedHandForward, Vector3.up).normalized;
        highestPoint = tpOrigin + projectedForward * maxPointingRange;

        for (int i = 0; i <= heightCheckRayCount; i++)
        {
            float angle = (i / heightCheckRayCount) * 90f;
            Quaternion rotationX = Quaternion.AngleAxis(angle, Vector3.Cross(projectedForward, Vector3.up));
            Vector3 rayDirection = rotationX * projectedForward;

            if (Physics.Raycast(tpOrigin, rayDirection, out RaycastHit hit, maxPointingRange, lm))
                highestPoint = hit.point;
        }

        SetToPosition(highestPoint);
    }

    /// <summary>
    /// Calculates and updates the teleportation target position based on the maximum range, surface clearance,  and
    /// teleportation mode, while validating the teleportation destination.
    /// </summary>
    /// <remarks>This method determines the teleportation target by performing raycasts to check for valid
    /// surfaces  and vertical clearance. It adjusts the teleportation target position and visual feedback based on  the
    /// teleportation mode and the validity of the destination.  If the teleportation mode is set to <see
    /// cref="TeleportMode.SnapPoints"/>, the method ensures the  target aligns with a valid snap point within the
    /// allowed angle. For other modes, it checks for  vertical clearance above the target position.  The method also
    /// updates the smoothing factor for horizontal movement and stores the last valid  height for fallback
    /// purposes.</remarks>
    private void MaxRangeTeleport()
    {
        Vector3 maxRangeCheckPoint = tpOrigin + calculatedHandForward * maxPointingRange;

        if (!Physics.Raycast(maxRangeCheckPoint, Vector3.down, out RaycastHit hitDistance, maxPointingRange, layerMask))
        {
            isValidTeleport = false;
            SetToPosition(new Vector3(teleportTarget.position.x, lastValidHeight, teleportTarget.position.z));
            ApplyCurveTransparent();
            setHorizontalSmoothingFactor = 20;
            return;
        }

        teleportCheckPoint = new Vector3(hitDistance.point.x, hitDistance.point.y + surfaceOffset, hitDistance.point.z);
        SetToPosition(teleportCheckPoint);

        if (teleportMode == TeleportMode.SnapPoints)
        {
            bool validSnap = isSnapped && CurrentSnapPointInAngle();
            isValidTeleport = validSnap;

            if (validSnap) ApplyCurveValid();
            else ApplyCurveInvalid(features.HasFlag(TeleportFeature.FadeOutCurveOnNoTarget));
        }
        else
        {
            noVerticalClearance = Physics.Raycast(teleportCheckPoint, Vector3.up, verticalClearance, layerMask);
            isValidTeleport = !noVerticalClearance;

            if (isValidTeleport) ApplyCurveValid();
            else ApplyCurveInvalidSolid();
        }

        setHorizontalSmoothingFactor = horizontalSmoothingFactor;
        lastValidHeight = teleportTarget.transform.position.y;
    }

    /// <summary>
    /// Determines whether the active snap point is within a specified distance and angle relative to the avatar's
    /// position and orientation.
    /// </summary>
    /// <remarks>This method checks both the proximity and angular alignment of the active snap point to
    /// determine if it is considered "current."</remarks>
    /// <returns><see langword="true"/> if the active snap point is within 3.5 units of the avatar's position  and within a
    /// 45-degree angle of the calculated hand forward direction; otherwise, <see langword="false"/>.</returns>
    private bool CurrentSnapPointInAngle()
    {
        if (Vector3.Distance(avatar.position, activeSnapPoint.transform.position) < 3.5f)
            return true;

        Vector3 toSnapPoint = activeSnapPoint.transform.position - activeOriginTransform.position;
        return Vector3.Angle(calculatedHandForward, toSnapPoint.normalized) <= 45;
    }

    /// <summary>
    /// Gradually adjusts the position of the object to simulate a fade-out effect along a calculated trajectory.
    /// </summary>
    /// <remarks>This method calculates a midpoint along the forward direction of the hand and sets the
    /// object's position to that point. It is intended to create a visual effect of the object fading out or moving
    /// away.</remarks>
    private void FadeOutCurve()
    {
        Vector3 maxRangePoint = tpOrigin + calculatedHandForward * (maxPointingRange / 2);
        SetToPosition(maxRangePoint);
    }

    /// <summary>
    /// Updates the target position of the object based on its snapping state.
    /// </summary>
    /// <remarks>If the object is snapped, the target position is set to the position of the active snap
    /// point,  adjusted by the specified surface offset. Otherwise, the target position is set to the provided 
    /// <paramref name="defaultTarget"/>.</remarks>
    /// <param name="defaultTarget">The default position to use if the object is not snapped.</param>
    private void SetToPosition(Vector3 defaultTarget)
    {
        targetPosition = !isSnapped
            ? defaultTarget
            : new Vector3(
                activeSnapPoint.transform.position.x,
                activeSnapPoint.transform.position.y + surfaceOffset,
                activeSnapPoint.transform.position.z
            );
    }

    #endregion

    #region Snapping

    /// <summary>
    /// Snaps the teleporter to the closest available snap point.
    /// </summary>
    /// <remarks>This method evaluates all available snap points and determines the best one to snap to.  If a
    /// suitable snap point is found, the teleporter's position is updated to match the snap point,  and any associated
    /// snap event is invoked. The method has no effect if there are no available snap points  or if the teleporter is
    /// inactive.</remarks>
    public void SnapToClosestSnapPoint()
    {
        if (availableSnapPoints.Count == 0 || !teleporterActive) return;
        XRTeleporterSnapPoint best = GetBestSnapPoint();

        if (!best) return;
        isSnapped = true;
        activeSnapPoint = best;

        activeSnapPoint?.onSnap?.Invoke();

        teleportCheckPoint = activeSnapPoint.transform.position;
        SetToPosition(teleportCheckPoint);
    }

    /// <summary>
    /// Determines the closest snappable teleportation snap point to the specified checkpoint.
    /// </summary>
    /// <remarks>This method evaluates all available snap points and selects the one with the smallest
    /// distance to the teleportation checkpoint, provided it is marked as snappable.</remarks>
    /// <returns>The <see cref="XRTeleporterSnapPoint"/> that is closest to the teleportation checkpoint and is marked as
    /// snappable; or <see langword="null"/> if no snappable points are available.</returns>
    private XRTeleporterSnapPoint GetBestSnapPoint()
    {
        float smallest = float.MaxValue;
        XRTeleporterSnapPoint best = null;

        foreach (var sp in availableSnapPoints)
        {
            if (!sp.isSnappable) continue;
            float d = Vector3.Distance(sp.transform.position, teleportCheckPoint);
            if (d < smallest) { smallest = d; best = sp; }
        }

        return best;
    }

    /// <summary>
    /// Unsnaps the object from its current snap point, if snap-based teleportation is enabled.
    /// </summary>
    /// <remarks>This method resets the object's snap state by invoking the unsnap action of the active snap
    /// point,  if one exists, and clearing the snap-related properties. If snap-based teleportation is disabled,  the
    /// method performs no action.</remarks>
    public void Unsnap()
    {
        if (!snapBasedTeleportation) return;

        activeSnapPoint?.onUnsnap?.Invoke();

        isSnapped = false;
        activeSnapPoint = null;
        useSnapPointForward = false;
    }

    #endregion

    #region Curve

    /// <summary>
    /// Generates a curved mesh between the teleport origin and the teleport target.
    /// </summary>
    /// <remarks>This method creates a visual representation of a curve by generating a mesh based on the
    /// positions of the teleport origin and target. The curve's height is determined by the angle between the teleport
    /// target's up direction and the calculated forward direction of the hand. The generated mesh is applied to the
    /// <see cref="MeshFilter"/> component attached to the same GameObject.  The curve is divided into segments, and
    /// each segment is calculated to form a smooth transition between the origin and target. The width of the curve and
    /// its height offset are also factored into the mesh generation.  This method assumes that the teleport target
    /// position has been initialized. If the target position is not initialized, the method exits without performing
    /// any operations.</remarks>
    private void GenerateCurve()
    {
        if (!hasInitializedTargetPosition) return;

        float angle = Vector3.Angle(teleportTarget.up, calculatedHandForward);
        height = Mathf.Lerp(maxCurveHeight, 0f, angle / 180f);

        var mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;

        var vertices = new List<Vector3>();
        var triangles = new List<int>();
        var uv = new List<Vector2>();

        Vector3 forward = (transform.InverseTransformPoint(teleportTarget.position) - transform.InverseTransformPoint(tpOrigin)).normalized;
        Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;

        for (int i = 0; i <= curveSegments; i++)
        {
            float t = i / (float)curveSegments;
            Vector3 pos = Vector3.Lerp(transform.InverseTransformPoint(tpOrigin), transform.InverseTransformPoint(teleportTarget.position), t);
            float heightOffset = Mathf.Sin(t * Mathf.PI) * height;

            Vector3 offset = right * (curveWidth * 0.5f);
            Vector3 upOffset = Vector3.up * heightOffset;

            Vector3 a = pos - offset + upOffset;
            Vector3 b = pos + offset + upOffset;

            vertices.Add(a); vertices.Add(b);
            uv.Add(new Vector2(0, t)); uv.Add(new Vector2(1, t));

            if (i <= 0) continue;
            int baseIdx = (i - 1) * 2;
            int curIdx = i * 2;

            triangles.Add(baseIdx); triangles.Add(baseIdx + 1); triangles.Add(curIdx);
            triangles.Add(curIdx); triangles.Add(baseIdx + 1); triangles.Add(curIdx + 1);
            triangles.Add(baseIdx + 1); triangles.Add(baseIdx); triangles.Add(curIdx);
            triangles.Add(curIdx + 1); triangles.Add(baseIdx + 1); triangles.Add(curIdx);
        }

        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.uv = uv.ToArray();
        mesh.RecalculateNormals();
    }

    /// <summary>
    /// Colors the curve to indicate a valid teleportation target.
    /// </summary>
    private void ApplyCurveValid() => ChangeCurveAppearance(defaultColor, Color.white, defaultColor, defaultTextureTiling, true);

    /// <summary>
    /// Colors the curve to indicate an invalid teleportation target, with an option to fade out the curve.
    /// </summary>
    /// <param name="fadeOutAllowed"></param>
    private void ApplyCurveInvalid(bool fadeOutAllowed)
    {
        if (!fadeOutAllowed)
            ChangeCurveAppearance(notValidColor, Color.white, notValidColor, defaultTextureTiling, true);
        else
            ChangeCurveAppearance(new Color(0, 0, 0, 0), new Color(0, 0, 0, 0), new Color(0, 0, 0, 0), defaultTextureTiling, false);
    }

    /// <summary>
    /// Colors the curve to indicate an invalid teleportation target with a solid appearance.
    /// </summary>
    private void ApplyCurveInvalidSolid() => ChangeCurveAppearance(notValidColor, Color.white, notValidColor, defaultTextureTiling, true);

    /// <summary>
    /// Colors the curve to be fully transparent, effectively hiding it.
    /// </summary>
    private void ApplyCurveTransparent() => ChangeCurveAppearance(new Color(0, 0, 0, 0), new Color(0, 0, 0, 0), new Color(0, 0, 0, 0), defaultTextureTiling, false);

    /// <summary>
    /// Colors the curve to indicate that the angle of the hand is too steep for teleportation.
    /// </summary>
    private void ApplyCurveAngleTooBig() => ChangeCurveAppearance(angleTooBigColor, Color.white, new Color(0, 0, 0, 0), new Vector2(1, 1.19f), false);

    /// <summary>
    /// Updates the appearance of the teleportation curve and target based on the current state and conditions.
    /// </summary>
    /// <remarks>The method adjusts the visual properties of the teleportation curve and target based on
    /// factors such as  whether the teleporter is active, the pointing range, and the angle of the hand. It also
    /// updates the  visibility of the teleportation target and applies the appropriate material properties to the curve
    /// and target renderers.</remarks>
    /// <param name="curveColor">The color to apply to the teleportation curve. This may be modified based on the current state.</param>
    /// <param name="targetOuterColor">The color to apply to the outer ring of the teleportation target. This may be modified based on the current
    /// state.</param>
    /// <param name="targetInnerColor">The color to apply to the inner ring of the teleportation target. This may be modified based on the current
    /// state.</param>
    /// <param name="tiling">The texture tiling to apply to the teleportation curve. This may be modified based on the current state.</param>
    /// <param name="targetVisible">A value indicating whether the teleportation target should be visible. This may be modified based on the current
    /// state.</param>
    private void ChangeCurveAppearance(Color curveColor, Color targetOuterColor, Color targetInnerColor, Vector2 tiling, bool targetVisible)
    {
        if (!teleporterActive)
        {
            targetVisible = false;
            curveColor = new Color(0, 0, 0, 0);
            targetOuterColor = new Color(0, 0, 0, 0);
            targetInnerColor = new Color(0, 0, 0, 0);
            tiling = defaultTextureTiling;
        }
        else if (distance < minPointingRange && Vector3.Angle(Vector3.up, calculatedHandForward) >= 4.5f)
        {
            targetVisible = true;
            curveColor = angleTooBigColor;
            targetOuterColor = Color.white;
            targetInnerColor = new Color(0.472f, 0.472f, 0.472f, .95f);
            tiling = defaultTextureTiling;
        }

        teleportTarget.gameObject.SetActive(targetVisible);

        curveRenderer.material.color = curveColor;

        if (useEmissiveMaterials) targetRenderer.materials[1].SetColor("_EmissionColor", targetOuterColor * 4.5f);
        else targetRenderer.materials[1].SetColor("_BaseColor", targetOuterColor);

        targetRenderer.materials[2].color = targetInnerColor;

        curveRenderer.material.mainTextureScale = tiling;
    }

    #endregion

    #region Utils

    // -- Properties for easier access to enum flags --

    public bool snapBasedTeleportation
    {
        get => teleportMode == TeleportMode.SnapPoints;
        set => teleportMode = value ? TeleportMode.SnapPoints : TeleportMode.Free;
    }

    public bool allowRotationWithJoysticks
    {
        get => features.HasFlag(TeleportFeature.RotationWithJoysticks);
        set => features = value ? (features | TeleportFeature.RotationWithJoysticks) : (features & ~TeleportFeature.RotationWithJoysticks);
    }

    public bool snapToHighestEdge
    {
        get => features.HasFlag(TeleportFeature.SnapToHighestEdge);
        set => features = value ? (features | TeleportFeature.SnapToHighestEdge) : (features & ~TeleportFeature.SnapToHighestEdge);
    }

    public bool fadeOutCurveOnNoTarget
    {
        get => features.HasFlag(TeleportFeature.FadeOutCurveOnNoTarget);
        set => features = value ? (features | TeleportFeature.FadeOutCurveOnNoTarget) : (features & ~TeleportFeature.FadeOutCurveOnNoTarget);
    }

    /// <summary>
    /// Configures the avatar by setting up interaction gates and disabling specific objects in the scene.
    /// </summary>
    /// <remarks>This method searches for objects with names containing "Loco" in the active scene, disables
    /// them,  and configures interaction gates with appropriate interactors for left and right hands.  If no matching
    /// objects are found, a warning is logged.</remarks>
    void SetupAvatar()
    {
        var objects = FindByNamePart("Loco", includeInactive: true, onlyActiveScene: true);

        if (objects.Count == 0)
        {
            Debug.LogWarning("Automatic Teleporter install failed - Please make sure your interaction rig is set up.");
            return;
        }

        foreach (var obj in objects)
            obj.SetActive(false);

        var gates = FindObjectsByType<HoverInteractorsGate>(FindObjectsInactive.Include, FindObjectsSortMode.None);

        for (int i = 0; i < gates.Length; i++) // first iteration: left, second: right
        {
            bool isLeft = i == 0;

            List<IInteractor> interactorsA = new() { FindInteractorByHand<PokeInteractor>(isLeft ? Handedness.Left : Handedness.Right, true) };
            gates[i].InjectInteractorsA(interactorsA);

            List<IInteractor> interactorsB = new() { FindInteractorByHand<RayInteractor>(isLeft ? Handedness.Left : Handedness.Right, true) ,
                FindInteractorByHand<DistanceHandGrabInteractor>(isLeft ? Handedness.Left : Handedness.Right, true)};
            gates[i].InjectInteractorsB(interactorsB);
        }
    }

    /// <summary>
    /// Sets up pose detection for hand tracking by configuring recognizers and event wrappers for left and right hand
    /// poses.
    /// </summary>
    /// <remarks>This method initializes and configures pose detection components for both left and right
    /// hands. It creates game objects for pose detection, associates them with the appropriate hand and  finger feature
    /// state providers, and sets up event wrappers to handle activation and deactivation  events for specific poses.
    /// The method also schedules the wiring of pose listeners to occur  in the next frame.</remarks>
    void SetupPoseDetection()
    {
        var hands = GameObject.Find("OVRHands")?.GetComponentsInChildren<Hand>();
        var providers = FindObjectsByType<FingerFeatureStateProvider>(FindObjectsSortMode.None);

        var toWire = new List<(ActiveStateUnityEventWrapper wrapper, bool isAimPose, bool isLeft)>();

        for (int i = 0; i < 2; i++)
        {
            bool isLeft = i == 0;

            var poseGO = new GameObject("PoseDetection_" + (isLeft ? "Left" : "Right"));
            poseGO.transform.SetParent(transform, worldPositionStays: false);

            for (int j = 0; j < 2; j++)
            {
                bool isAimPose = j == 0;

                var shape = poseGO.AddComponent<ShapeRecognizerActiveState>();
                shape.InjectHand(isLeft ? hands[0] : hands[1]);
                shape.InjectFingerFeatureStateProvider(isLeft ? providers[0] : providers[1]);

                var shapesList = new List<ShapeRecognizer>();
                var candidate = isAimPose ? teleportAimPose : teleportConfirmPose;
                if (candidate) shapesList.Add(candidate);
                shape.InjectShapes(shapesList.ToArray());

                var wrapper = poseGO.AddComponent<ActiveStateUnityEventWrapper>();
                wrapper.InjectActiveState(shape);
                wrapper.InjectOptionalEmitOnFirstUpdate(false);

                wrapper.InjectOptionalWhenActivated(new UnityEngine.Events.UnityEvent());
                wrapper.InjectOptionalWhenDeactivated(new UnityEngine.Events.UnityEvent());

                toWire.Add((wrapper, isAimPose, isLeft));
            }
        }

        StartCoroutine(WirePoseListenersNextFrame(toWire));
    }

    /// <summary>
    /// Registers event listeners for a collection of pose-related state wrappers on the next frame.
    /// </summary>
    /// <remarks>This method defers the registration of event listeners until the next frame to ensure proper
    /// initialization. For each item in the list: <list type="bullet"> <item> <description>When the wrapper is
    /// activated, it triggers either <see cref="SetTeleporterAimPoseActive"/> or <see
    /// cref="TeleporterConfirmPoseDetected"/> based on whether the pose is an aim pose.</description> </item> <item>
    /// <description>When the wrapper is deactivated, it triggers <see cref="SetTeleporterAimPoseInactive"/> if the pose
    /// is an aim pose.</description> </item> </list></remarks>
    /// <param name="items">A list of tuples, where each tuple contains: <list type="bullet"> <item> <description><paramref
    /// name="wrapper"/>: The <see cref="ActiveStateUnityEventWrapper"/> to wire listeners to.</description> </item>
    /// <item> <description><paramref name="isAimPose"/>: A boolean indicating whether the pose is an aim
    /// pose.</description> </item> <item> <description><paramref name="isLeft"/>: A boolean indicating whether the pose
    /// is associated with the left side.</description> </item> </list></param>
    /// <returns></returns>
    IEnumerator WirePoseListenersNextFrame(List<(ActiveStateUnityEventWrapper wrapper, bool isAimPose, bool isLeft)> items)
    {
        yield return null;

        foreach (var item in items)
        {
            var w = item.wrapper;
            bool rightSide = !item.isLeft;

            w.WhenActivated.AddListener(() =>
            {
                if (item.isAimPose) SetTeleporterAimPoseActive(rightSide);
                else TeleporterConfirmPoseDetected();
            });

            w.WhenDeactivated.AddListener(() =>
            {
                if (item.isAimPose) SetTeleporterAimPoseInactive(rightSide);
            });
        }
    }

    /// <summary>
    /// Finds all <see cref="GameObject"/> instances whose names contain the specified substring.
    /// </summary>
    /// <remarks>If <paramref name="onlyActiveScene"/> is <see langword="true"/>, the method filters results
    /// to include only <see cref="GameObject"/> instances in the currently active scene. If <paramref
    /// name="onlyActiveScene"/> is <see langword="false"/>, the method includes objects from all valid
    /// scenes.</remarks>
    /// <param name="namePart">The substring to search for within the names of <see cref="GameObject"/> instances. The comparison is
    /// case-insensitive.</param>
    /// <param name="includeInactive">A value indicating whether to include inactive <see cref="GameObject"/> instances in the search. <see
    /// langword="true"/> to include inactive objects; otherwise, <see langword="false"/>.</param>
    /// <param name="onlyActiveScene">A value indicating whether to restrict the search to the currently active scene. <see langword="true"/> to
    /// search only in the active scene; otherwise, <see langword="false"/>.</param>
    /// <returns>A list of <see cref="GameObject"/> instances whose names contain the specified substring. If no matching objects
    /// are found, an empty list is returned.</returns>
    public static List<GameObject> FindByNamePart(string namePart, bool includeInactive = true, bool onlyActiveScene = true)
    {
        var opts = includeInactive ? FindObjectsInactive.Include : FindObjectsInactive.Exclude;
        var all = FindObjectsByType<GameObject>(opts, FindObjectsSortMode.None);
        var cmp = StringComparison.OrdinalIgnoreCase;

        if (onlyActiveScene)
        {
            var active = SceneManager.GetActiveScene();
            return all.Where(go =>
                    go.scene == active &&
                    go.name.IndexOf(namePart, cmp) >= 0)
                .ToList();
        }

        return all.Where(go =>
                go.scene.IsValid() &&
                go.name.IndexOf(namePart, cmp) >= 0)
            .ToList();
    }

    /// <summary>
    /// Finds an interactor of the specified type associated with the given hand side.
    /// </summary>
    /// <remarks>This method searches through all objects of type <typeparamref name="T"/> in the scene,
    /// including inactive objects if applicable. If no matching interactor is found, an error is logged to the
    /// console.</remarks>
    /// <typeparam name="T">The type of the interactor to find. Must implement <see cref="IInteractor"/> and inherit from <see
    /// cref="MonoBehaviour"/>.</typeparam>
    /// <param name="side">The hand side (<see cref="Handedness"/>) to search for. This determines whether the interactor is associated
    /// with the left or right hand.</param>
    /// <param name="searchForHandRef">A boolean value indicating whether to restrict the search to interactors with a <see cref="HandRef"/> component.
    /// If <see langword="true"/>, only interactors with a <see cref="HandRef"/> matching the specified hand side will
    /// be considered. If <see langword="false"/>, interactors with either a <see cref="HandRef"/> or a <see
    /// cref="ControllerRef"/> matching the specified hand side will be considered.</param>
    /// <returns>The first interactor of type <typeparamref name="T"/> that matches the specified hand side and search criteria,
    /// or <see langword="null"/> if no matching interactor is found.</returns>
    public static T FindInteractorByHand<T>(Handedness side, bool isHand = false) where T : MonoBehaviour, IInteractor
    {
        var interactors = FindObjectsByType<T>(true ? FindObjectsInactive.Include : FindObjectsInactive.Exclude, FindObjectsSortMode.None);

        foreach (var it in interactors)
        {
            if (isHand)
            {
                if (it.TryGetComponent<HandRef>(out var handRef) && GetHandRefSide(handRef) == side) return it;
                continue;
            }

            if ((it.TryGetComponent<HandRef>(out var hr) && GetHandRefSide(hr) == side) || (it.TryGetComponent<ControllerRef>(out var cr) && GetControllerRefSide(cr) == side)) return it;
        }

        Debug.LogError($"FindInteractorByHand<{typeof(T).Name}>: couldn't find interactor for side {side} (isHand={isHand}).");
        return null;
    }

    /// <summary>
    /// Determines the handedness (left or right) of the specified controller reference.
    /// </summary>
    /// <param name="reference">The hand reference object used to determine the handedness.</param>
    /// <returns><see cref="Handedness.Right"/> if the reference is associated with the right controller;  otherwise, <see
    /// cref="Handedness.Left"/>.</returns>
    public static Handedness GetControllerRefSide(ControllerRef reference)  // don't ask - its the only way this works outside of play mode
    {
        return reference.transform.parent.parent.parent.name.Contains("Right") ? Handedness.Right : Handedness.Left;
    }

    /// <summary>
    /// Determines the handedness (left or right) of the specified hand reference.
    /// </summary>
    /// <param name="reference">The hand reference object used to determine the handedness.</param>
    /// <returns><see cref="Handedness.Right"/> if the reference is associated with the right hand;  otherwise, <see
    /// cref="Handedness.Left"/>.</returns>
    public static Handedness GetHandRefSide(HandRef reference)  // same here
    {
        return reference.transform.parent.parent.parent.name.Contains("Right") ? Handedness.Right : Handedness.Left;
    }

    /// <summary>
    /// Enumerates the available hands in the context.
    /// </summary>
    /// <remarks>This method yields the hands in a predefined order, starting with the right hand  followed by
    /// the left hand. The enumeration is lazy and does not create a new collection.</remarks>
    /// <returns>An <see cref="IEnumerable{T}"/> of <see cref="HandCtx"/> objects,  where each object represents a hand in the
    /// context.</returns>
    private IEnumerable<HandCtx> EnumerateHands()
    {
        yield return _right;
        yield return _left;
    }

    /// <summary>
    /// Retrieves the hand context for the specified side.
    /// </summary>
    private HandCtx GetHand(bool rightSide) => rightSide ? _right : _left;

    /// <summary>
    /// Retrieves the opposite hand context of the specified hand.
    /// </summary>
    private HandCtx GetOtherHand(HandCtx h) => h.Side == Handedness.Right ? _left : _right;

    /// <summary>
    /// Draws debug rays representing the orientation and key directional vectors of a hand transform.
    /// </summary>
    /// <remarks>This method is intended for debugging purposes and uses <see cref="Debug.DrawRay"/> to
    /// visualize the directional vectors of the hand in the Unity Editor. The method does nothing if <paramref
    /// name="handTransform"/> is <see langword="null"/>.</remarks>
    /// <param name="handTransform">The transform of the hand for which the debug rays will be drawn. Must not be <see langword="null"/>.</param>
    /// <param name="forwardColor">The color of the ray representing the forward direction of the hand.</param>
    /// <param name="projectedColor">The color of the ray representing the forward direction of the hand projected onto the horizontal plane.</param>
    /// <param name="rightColor">The color of the ray representing the right or up vector of the hand, depending on the hand's orientation.</param>
    /// <param name="upColor">The color of the ray representing the global upward direction.</param>
    private void DrawHandDebugRays(Transform handTransform, Color forwardColor, Color projectedColor, Color rightColor, Color upColor)
    {
        if (handTransform == null || debugMode == false) return;

        Vector3 handForward = CalculateHandForward();
        Vector3 projectedForward = Vector3.ProjectOnPlane(handForward, Vector3.up);
        Vector3 upVector = handTransform == _right.Hand ? handTransform.up : -handTransform.up;

        Debug.DrawRay(handTransform.position, handForward, forwardColor);
        Debug.DrawRay(handTransform.position, projectedForward, projectedColor);
        Debug.DrawRay(handTransform.position, upVector, rightColor);
        Debug.DrawRay(handTransform.position, Vector3.up, upColor);
    }

    #endregion
}

public enum TeleportMode { Free, SnapPoints }

[Flags]
public enum TeleportFeature
{
    None = 0,
    RotationWithJoysticks = 1 << 0,
    SnapToHighestEdge = 1 << 1,
    FadeOutCurveOnNoTarget = 1 << 2
}

public class HandCtx
{
    public Handedness Side;
    public Transform Controller;
    public Transform Hand;
    public IInteractor GrabInteractor, PokeInteractor, DistanceGrabInteractor, RayInteractor;
    public IInteractor HandGrabInteractor, HandPokeInteractor, HandDistanceGrabInteractor, HandRayInteractor;
    public OVRInput.Controller OvrController;

    public bool AimingActive;
    public bool AimQueued;
    public bool DeactivatedByHover;

    public bool IsSelecting => (GrabInteractor != null && GrabInteractor is GrabInteractor grab && grab.State == InteractorState.Select) ||
                               (PokeInteractor != null && PokeInteractor is PokeInteractor poke && poke.State == InteractorState.Select);
    public bool IsHovering => (GrabInteractor != null && GrabInteractor is GrabInteractor grab && grab.State == InteractorState.Hover) ||
                               (PokeInteractor != null && PokeInteractor is PokeInteractor poke && poke.State == InteractorState.Hover);

    public void GetInteractors()
    {
        GrabInteractor = XRTeleporter.FindInteractorByHand<GrabInteractor>(Side, false);
        PokeInteractor = XRTeleporter.FindInteractorByHand<PokeInteractor>(Side, false);
        DistanceGrabInteractor = XRTeleporter.FindInteractorByHand<DistanceGrabInteractor>(Side, false);
        RayInteractor = XRTeleporter.FindInteractorByHand<RayInteractor>(Side, false);
        HandGrabInteractor = XRTeleporter.FindInteractorByHand<HandGrabInteractor>(Side, true);
        HandPokeInteractor = XRTeleporter.FindInteractorByHand<PokeInteractor>(Side, true);
        HandDistanceGrabInteractor = XRTeleporter.FindInteractorByHand<DistanceHandGrabInteractor>(Side, true);
        HandRayInteractor = XRTeleporter.FindInteractorByHand<RayInteractor>(Side, true);
    }
}