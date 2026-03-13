# Skeletal Motion Interpolation

# Demo Video


![Motion Interpolation Demo](./BezierEuler-ezgif.com-video-to-gif-converter.gif)


![Motion Interpolation Demo](./ezgif.com-video-to-gif-converter.gif)


![Motion Interpolation Demo](./ezgif.com-video-to-gif-converter copy.gif)


<br>
<br>

# What’s Implemented

# 1) Angle Representation Conversions
- Where: `interpolator.cpp` (`Euler2Rotation()`, `Euler2Quaternion()`, `Quaternion2Euler()`).
- What:
  - Implements the mathematical conversion from 3D Euler angles to a $3 \times 3$ rotation matrix using trigonometric combinations.
  - Acts as a bridge to convert raw Euler angle data from motion capture files into Quaternions, avoiding gimbal lock and preparing the data for robust spherical interpolation, then converting back to Euler for the final posture rendering.

# 2) Spherical Linear Interpolation (SLERP)
- Where: `interpolator.cpp` (`Slerp()`).
- What:
  - Implements shortest-path interpolation between two Quaternions on the unit sphere.
  - Computes the dot product (`cosTheta`) to find the angle $\theta$ between rotations, clamping the value to $[-1, 1]$ to prevent `acos()` domain errors.
  - Dynamically calculates weights (`sin((1.0 - t) * theta) / sinTheta`).
  - Includes a fallback mechanism: safely degenerates to standard LERP when the rotation angle is close to zero, preventing divide-by-zero errors.


# 3) De Casteljau's Algorithm
- Where: `interpolator.cpp` (`DeCasteljauEuler()`, `DeCasteljauQuaternion()`).
- What:
  - Recursively evaluates cubic Bezier curves for any given parameter $t \in [0, 1]$.
  - For standard vectors (root positions and Euler angles), it repeatedly applies linear interpolation (LERP) across the 4 control points.
  - For rotational data, it recursively applies SLERP across 4 Quaternions, generating perfectly smooth, higher-order spherical trajectories.


# 4) Linear Motion Interpolation
- Where: `interpolator.cpp` (`LinearInterpolationEuler()`, `LinearInterpolationQuaternion()`).
- What:
  - Generates $N$ missing frames between two keyframes.
  - Linearly interpolates the character's root position.
  - For skeletal joints, offers two modes: directly LERPing Euler angles (which can cause unnatural rotations), or safely transforming to Quaternions, performing SLERP, and converting back to guarantee accurate, constant-velocity joint rotations.

# 5) Bezier Motion Interpolation
- Where: `interpolator.cpp` (`BezierInterpolationEuler()`, `BezierInterpolationQuaternion()`).
- What:
  - Upgrades the motion interpolation from linear to cubic Bezier to create fluid, non-robotic skeletal animations.
  - Computes optimal control points ($a_n, b_n$) by looking at adjacent keyframes (`pPrev`, `pStart`, `pEnd`, `pNextNext`) to ensure $C^1$ continuity (smooth velocity transitions) across keyframe boundaries.
  - Uses the implemented `DeCasteljauEuler` and `DeCasteljauQuaternion` functions to evaluate the exact intermediate poses frame by frame.
