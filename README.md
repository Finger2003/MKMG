# 3D CAD & Geometry Modeling Engine

![Hero Scene - Sailboat Model](https://github.com/user-attachments/assets/522141ab-f6f6-410e-a2b0-65bba7a4c88e)
*A complex sailboat model built using intersecting C0 and C2 bicubic Bezier surfaces.*

A comprehensive, high-performance 3D modeling and geometry processing engine built from scratch in C++ and DirectX 11. This project features a custom SIMD-accelerated mathematics library, an adaptive CPU ray-caster, and a fully interactive CAD application capable of advanced operations like numerical surface intersections, GPU-accelerated trimming, and $C^1$ topological hole-filling.

## ✨ Visual Showcase

### Surface Intersections & GPU Trimming
![Trimmed Torus](https://github.com/user-attachments/assets/854a1384-4a60-40d8-8073-8406b9f6e4d5)
*Precise numerical intersection curve found via Newton's method, with non-destructive GPU pixel-shader trimming.*

![Gregory Patch](https://github.com/user-attachments/assets/7f895159-b55d-48c2-a6b3-e6b21fffce62)
*Automatic detection of 3-sided surface gaps, seamlessly sealed using Gregory Patches to maintain $C^1$ continuity with surrounding Bezier surfaces.*

### Adaptive CPU Ray-Casting
<video src="https://github.com/user-attachments/assets/0a258127-e6c2-4d19-a35b-9937da28495e" autoplay loop muted playsinline width="100%">
  Your browser does not support the video tag.
</video>
*Implicit ellipsoid rendering on the CPU. The resolution dynamically scales down during camera rotation to maintain high framerates, instantly refining when static.*

### Anaglyph 3D Stereoscopy
![Stereoscopy](https://github.com/user-attachments/assets/641f1f90-d34d-4a0e-933d-49e26a49ac17)
*Built-in Anaglyph 3D rendering (Red/Cyan) with real-time configurable eye separation and focal depth.*

---

## 🏗️ Solution Structure

The repository is divided into highly decoupled, specialized projects:

* **CAD (`CAD.vcxproj`)**: The primary DirectX 11 application. A fully interactive 3D modeling environment featuring parametric surfaces, spline interpolation, and advanced geometry processing.
* **Ellipsoid (`Ellipsoid.vcxproj`)**: A standalone adaptive CPU rendering application. It visualizes an ellipsoid via ray-casting its implicit quadratic affine equation ($w^T D_M' w = 0$), calculating analytic surface normals for specular Phong illumination.
* **MathLib (`MathLib.vcxproj`)**: The core mathematics engine implementing hardware-level SIMD optimizations using AVX intrinsics (`__m128` and `__m256d`). It features custom 4x4 matrix inversion, matrix-quaternion conversions, and a linear system solver utilizing Gaussian elimination with partial pivoting.
* **MathLibTest**: Unit testing framework validating the custom algebra, matrix solvers, and SIMD registers.
* **ImGuiLib**: The immediate mode GUI library integrated directly into the solution tree.

## 🚀 Key Features

* **Architecture & Rendering Pipeline**: Strict separation of topological data from geometric coordinates.
  * *Observer Pattern Dependency Graph*: Implements an `IPointDependent` interface. Shared control points act as subjects; modifying or merging a single point automatically cascades updates to all connected topological dependents (curves, surfaces, patches).
  * *Adaptive GPU Tessellation*: The CPU only dispatches control points, while the GPU dynamically generates the dense geometry to prevent visual faceting and preserve smooth curvature. Utilizes Geometry Shaders for curves and Hardware Tessellation (Hull/Domain shaders) to generate parametric grids for Bezier surfaces and Gregory Patches.
* **Surface Intersections & GPU Trimming**: Calculates exact numerical intersections between any combination of supported geometries (C0 and C2 bicubic patches, flat or cylindrical topologies, Torus primitives, and surface self-intersections):
  * *Global Candidate Search*: Hierarchical AABB pruning via recursive quadtree parameter subdivision (or interactive 3D cursor seeding) to isolate potential intersection zones.
  * *Starting Point Convergence*: Adaptive-step gradient descent minimizing 4D Euclidean distance to reach exact surface contact.
  * *Curve Marching & Tracing*: Predictor-corrector tracking along the tangent vector ($T = N_1 \times N_2$) using a 4D Newton-Raphson solver with partial pivoting, adaptive step-halving, and parametric boundary/seam trapping.
  * *Curve Decimation & Cyclic Solving*: Decimates dense intersection polylines using an iterative Ramer-Douglas-Peucker (RDP) algorithm, then constructs smooth $C^2$ interpolating Bezier curves using the cyclic Sherman-Morrison solver for closed loops.
  * *Non-Destructive Trimming*: Reconstructs closed UV contours across periodic seams and renders hardware stencil masks into dynamic trim textures, allowing real-time switching of the discarded surface side without recomputing the intersection curve.
  * *Robust Self-Intersection Handling*: Avoids trivial identity solutions ($u_1 = u_2, v_1 = v_2$) via UV domain separation during hierarchical pruning, dynamic parameter distance thresholds during seed selection, and degenerate diagonal-sink rejection during Newton marching.
* **Geometry Operations**: Point merging and topology management.
  * *Graph-Based Hole Detection*: Constructs an adjacency graph from surface boundary edges and filters out internal seams. It then performs graph traversal to automatically detect and extract 3-cycles (holes), enabling seamless Gregory Patch generation.
* **Multi-Segment Curves**: Supports the creation and editing of multi-segment 3rd degree Bezier (C0) curves, B-Spline (C2) curves, and Interpolating C2 splines. 
  * *Basis Conversion*: Features real-time bi-directional conversion between B-Spline and Bernstein bases, calculating and rendering "virtual" Bernstein control points without polluting the scene hierarchy.
  * *Inverse Virtual Editing*: Allows users to interactively drag dynamically generated "virtual" Bernstein points on a B-Spline; the engine automatically solves the inverse transformations to update the underlying De Boor control points in real-time.
  * *Tridiagonal Solving*: The interpolating splines guarantee $C^2$ continuity by solving tridiagonal linear systems of polynomial equations using the efficient $O(N)$ Thomas Algorithm.
* **Parametric Surfaces**: Generation of C0 and C2 bicubic Bezier surfaces in both flat and cylindrical topologies.
* **Mathematical Evaluation**: Evaluates exact spatial coordinates, tangents, and analytical partial derivatives ($du, dv$) for curves and surfaces utilizing the recursive De Casteljau and De Boor algorithms.
* **Serialization**: Save and load complete scene hierarchies, object states, and dependent point references using JSON.
* **User Interface Polish**: The ImGui-based hierarchy panel features case-insensitive object filtering and double-click inline renaming for efficient scene management.

## 💻 Technologies & Requirements

* **Language**: C++20
* **Graphics API**: DirectX 11 via WinAPI
* **Hardware Requirements**: CPU with **AVX instruction set support** required. 
  * The CAD application utilizes `__m128` registers.
  * The Ellipsoid application utilizes `__m256d` registers.
* **Dependencies**: 
  * `ImGui` (Included in-tree)
  * `nlohmann.json` (Installed via NuGet for serialization)

## 🛠️ Build Instructions

1. Clone the repository.
2. Open `MG.slnx` in **Visual Studio 2022**.
3. Ensure NuGet packages are restored (Right-click Solution -> Restore NuGet Packages).
4. Set the build platform to **`x64`**.
5. Set `CAD` or `Ellipsoid` as the Startup Project and hit **Build/Run**.

---

## ⌨️ User Guide & Key Controls

The application features a robust set of viewport and editing controls, designed around a 3D Cursor paradigm.

### Camera Navigation
* **Middle Mouse Button (Hold)**: Orbit the camera around the focal point.
* **Right Mouse Button (Hold)**: Pan the camera.
* **Mouse Wheel**: Zoom the camera in and out.

### Selection & 3D Cursor
* **Left Mouse Button (LMB) on Object**: Selects the clicked point or object.
* **LMB in Empty Space (Action = None)**: Clears the current selection and moves the 3D cursor to the clicked location.
* **Ctrl + LMB on Point**: Toggles the selection state of the specific point.
* **Ctrl + LMB in Empty Space**: Places the 3D cursor at the clicked location without triggering the active interactive action or clearing the current selection.
* **Shift + LMB (in UI)**: Selects a range of items in the object hierarchy list.
* **Hold B + Drag**: Box selection to highlight multiple objects simultaneously. *(Hold **Ctrl** to subtract from selection, or **Shift** to add to selection)*.

### Transformations & Editing
* **LMB in Empty Space (Action Active)**: Initiates the selected interactive action (Translate, Rotate, or Scale) on the selected objects. Moving the mouse applies the transformation.
  * *Default Behavior*: Transformations are applied relative to the mass center of the selected objects.
  * *Hold Shift*: Transformations are applied relative to the 3D cursor's position.
  * *Free Arcball Rotation*: When the Arcball rotation action is active, maps 2D mouse movements onto a virtual 3D sphere for intuitive free-axis rotation.
* **Hold P + LMB**: Places a new geometric Point precisely at the 3D cursor's location.

### Tools & File Management
* **M**: Merge selected points into a single averaged point (automatically updates dependent curves/surfaces).
* **G**: Seal a detected 3-sided hole by generating a Gregory patch.
* **I**: Open the surface intersection parameters popup.
* **Ctrl + S**: Save the current scene.
* **Ctrl + Shift + S**: Save the scene as a new file.
* **Ctrl + O**: Open and load a saved scene.
