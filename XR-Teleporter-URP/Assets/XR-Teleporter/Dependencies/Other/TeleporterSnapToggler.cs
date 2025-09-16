using UnityEngine;
using static OVRInput;

public class TeleporterSnapToggler : MonoBehaviour
{
    XRTeleporter teleporter;
    bool useSnap = false;

    private void Start()
    {
        teleporter = FindFirstObjectByType<XRTeleporter>();
        teleporter.teleportMode = TeleportMode.Free;
    }

    private void Update()
    {
        if(GetDown(Button.One))
        {
            useSnap = !useSnap;
            teleporter.teleportMode = useSnap ? TeleportMode.SnapPoints : TeleportMode.Free;
        }
    }
}
