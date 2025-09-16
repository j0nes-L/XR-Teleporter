# XR-Teleporter Package

An XR Locomotor with extended customization which offers free- and snap point based teleportation as well as hand tracking support. Please note that this package only works with URP.

Install via **Unity Package Manager** with: `https://github.com/j0nes-L/XR-Teleporter.git?path=/XR-Teleporter-Package`  
(+ -> Install Package from git URL)  

This package comes with a a **Sample Scene**, presenting the basic capabilities of the 2 main Components: 
- `XRTeleporter`
- `XRTeleporterSnapPoint`

These components can be inserted into your scene from the `Runtime/Prefabs` Directory of the package.

---

**! IMPORTANT !**  

When you first drag the `XRTeleporter` prefab into a scene with a configured `OVRCameraRig` (Interaction Rig installed), the script will automatically disable all locomotion components of your rig. 

Additionally you will have to manually add your desired `ShapeRecognizer` assets for the hand pose recognition. If you have the Meta AIO SKD installed, you can find them under:  
`Packages/Meta XR Interaction SDK Essentials > Runtime > Sample > Poses > Shapes` (or just create your own)

Now go and have fun with it! I'll try updating it from time to time!
~ Jonas
