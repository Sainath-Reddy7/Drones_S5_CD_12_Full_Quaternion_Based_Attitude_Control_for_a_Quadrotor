<div align="center">
  <figure>
    <img src="Misc/Images/Amritalogo.png" alt="AmritaLogo" width="500"> <br>
  </figure>
</div>

# CD12_Full_Quaternion_Based_Attitude_Control_for_a_Quadrotor

# Group & Team Members
<table>
  <tr>
 <td colspan="3">Group - 12</td>

  </tr>
  <tr class="header">
    <td>Name</td>
    <td>Roll No</td>
    <td>Email-Id</td>
  </tr>
  <tr>
    <td>Manohar Paturi</td>
    <td>CB.SC.U4AIE24339</td>
    <td>cb.sc.u4aie24339@cb.students.amrita.edu</td>
  </tr>
  <tr>
    <td>K Pushpak</td>
    <td>CB.SC.U4AIE24328</td>
    <td>cb.sc.u4aie24328@cb.students.amrita.edu</td>
  </tr>
  <tr>
    <td>Sai Krishna</td>
    <td>CB.SC.U4AIE24308</td>
    <td>cb.sc.u4aie24308@cb.students.amrita.edu</td>
  </tr>
  <tr>
    <td>B Sainath</td>
    <td>CB.SC.U4AIE24309</td>
    <td>cb.sc.u4aie24309@cb.students.amrita.edu</td>
  </tr>
  <tr>
    <td>Vishal</td>
    <td>CB.SC.U4AIE24363</td>
    <td>cb.sc.u4aie24363@cb.students.amrita.edu</td>
  </tr>
</table>

## Abstract

Attitude control of a quadrotor is most commonly performed by converting the vehicle's orientation into Euler angles and regulating each angle separately. Although intuitive, Euler angles suffer from an inherent geometric singularity known as **gimbal lock**, are computationally expensive due to repeated trigonometric evaluation, and reintroduce exactly the nonlinearities that quaternion-based estimation had avoided. The direction cosine matrix (DCM) removes the singularity but carries nine coupled states under an orthogonality constraint.

This project is a complete, tested, open-source reproduction of Fresk and Nikolakopoulos's nonlinear **P² (proportional-squared) quaternion attitude controller**, in which both the quadrotor's attitude model and the control law are implemented **entirely in quaternion space**, with no Euler-angle or DCM computation anywhere in the plant or the control loop. Every equation of the paper, (1)–(21), is implemented function-for-function, including a literal preservation of the paper's own sign convention on the rotational kinematics (eq. 18), which differs from the general body-frame quaternion derivative (eq. 7).

All three of the paper's benchmark scenarios — step, sinusoidal tracking, and 360° flip — are reproduced quantitatively and qualitatively under the paper's own measurement noise and torque bounds, including the absence of any gimbal-lock artifact through the full-rotation flip, matching the paper's central claim.

## Base Paper Metadata

- Conference: 2013 European Control Conference (ECC)
- Location: Zürich, Switzerland, July 17–19, 2013
- Year: 2013
- Pages: 3864–3869
- DOI: 10.23919/ECC.2013.6669617
- Publisher: European Control Association (EUCA)
- Authors: Emil Fresk, George Nikolakopoulos

## Base Paper Discussion

The paper proposes a full quaternion based control scheme for the attitude control problem of a quadrotor. Both the quadrotor's attitude model and the proposed non-linear Proportional-squared ($P^2$) control algorithm are implemented in the quaternion space, **without any transformations or calculations in the Euler-angle space or the DCM**. The authors observe that although quaternions are commonly used for attitude *estimation*, most published quadrotor *controllers* still convert the quaternion error back into an Euler-angle or axis-angle error before applying a control law — reintroducing exactly the nonlinearities and singularities quaternions were meant to avoid.

The paper builds the quaternion algebra (eqs. 1–16), combines the body-frame quaternion kinematics with the Newton–Euler rotational dynamics into a quaternion-form plant (eqs. 17–18), and designs a derivative-free non-linear $P^2$ controller acting directly on the quaternion error (eqs. 19–21). The controller is evaluated in simulation on three fundamental tracking test cases: a constant-rotation **step** input, a periodic **sinusoidal** reference, and a complex **360° flip** maneuver, the last specifically chosen to stress-test the absence of singularities through a full rotation.

However, the paper reports continuous-time-style simulation results without specifying a digital control-loop sample rate, and its evaluation vehicle is attitude-only: it treats the control-signal-to-torque relation as the identity and never introduces mass, geometry, rotors, or a mixer.

Our project reproduces the paper end-to-end as a tested software-in-the-loop simulation: every equation (1)–(21) is implemented function-for-function in Python, the paper's under-specified details (noise distribution, scenario timings, DCM convention) are resolved with documented defaults rather than silent choices, and all three benchmark scenarios are reproduced under the paper's own gains, inertia, torque bounds and measurement noise.

## Problem Statement

Euler-angle attitude representations suffer gimbal lock when two rotational axes align, are computationally expensive due to repeated trigonometric evaluation, and this cost compounds when a Jacobian of the system is required for control or estimation. Controllers that convert the quaternion error back into Euler angles inherit exactly these defects.

A complete implementation is required in which the plant dynamics, the error computation, and the control law all operate on the quaternion directly, with Euler angles used only for plotting and logging.

## Project Objectives

1. Implement the complete quaternion algebra of the paper — representation, multiplication, norm, conjugate, inverse, derivatives, rotations, DCM, and Euler conversions (eqs. 1–16) — with Euler angles restricted to plotting and logging only.
2. Implement the quaternion-form rigid-body attitude plant (eqs. 17–18), preserving the paper's own leading-minus sign convention on $\dot q$.
3. Implement the non-linear $P^2$ attitude controller (eqs. 19–21) with the paper's tuned gains $P_q=20$, $P_\omega=4$ and $\pm 4$ N·m torque saturation.
4. Reproduce the paper's three benchmark scenarios — step response, sinusoidal tracking, and 360° flip — under the paper's full 0.1-amplitude measurement noise, matching both the quantitative figures (settling time, phase lag) and the qualitative claims (small overshoot, singularity-free full rotation).

## Brief Project Explanation

This project reproduces the full quaternion based attitude controller for a quadrotor as a Python software-in-the-loop (SITL) simulation. The quadrotor's attitude is carried entirely by a unit quaternion $q$; the plant integrates the paper's quaternion kinematics together with the Newton–Euler rotational dynamics, and the controller feeds back the vector part of the quaternion error and the measured body rates to produce torque.

The quaternion error between a reference quaternion $q_{ref}$ and the measured quaternion $q_m$ is computed directly in quaternion space, and a non-linear $P^2$ control law — an outer attitude loop with gain $P_q$ and an inner rate loop with gain $P_\omega$ — generates the driving torque without any derivative action or integral term. Because the plant is a double integrator from torque to attitude, the steady-state error goes to zero without an integrator.

The overall control loop is:

```text
        q_ref
          │
          ▼
   q_err = q_ref ⊗ q_m*         -- eq. (19)
          │
          │  take vector part
          ▼
   Axis_err                     -- eq. (20)
          │
          ▼
   τ = -P_q·Axis_err - P_ω·ω_m  -- eq. (21)
          │
          │  saturate at ±4 N·m
          ▼
   Quadrotor attitude plant     -- eqs. (17)-(18)
          │
          ▼
   q_m , ω_m   (with 0.1 noise)
          │
          └──►  fed back to q_err
```

## Methodology

### 1. Quaternion Representation

A quaternion is a hyper complex number of rank 4. The units $q_1$ to $q_3$ form the **vector part**, while $q_0$ is the **scalar part**. It can be represented in hyper-complex form (eq. 1) or vector form (eq. 2):

$$
q = q_0 + q_1 i + q_2 j + q_3 k
$$

$$
q=\begin{bmatrix}q_0&q_1&q_2&q_3\end{bmatrix}^T
$$

Throughout this project the scalar-first vector form is used, with all quaternions constrained to unit length.

---

### 2. Quaternion Multiplication

Multiplication of two quaternions $p, q$ is performed by the Kronecker product, denoted $\otimes$. If $p$ represents one rotation and $q$ represents another, $p \otimes q$ represents the **combined rotation**. Quaternion multiplication is **non-commutative**, just as rotations are:

$$
p \otimes q =
\begin{bmatrix}
p_0 q_0 - p_1 q_1 - p_2 q_2 - p_3 q_3 \\
p_0 q_1 + p_1 q_0 + p_2 q_3 - p_3 q_2 \\
p_0 q_2 - p_1 q_3 + p_2 q_0 + p_3 q_1 \\
p_0 q_3 + p_1 q_2 - p_2 q_1 + p_3 q_0
\end{bmatrix}
$$

The same product can be written as a matrix–vector multiplication using either a left- or a right-multiplication matrix, $p\otimes q = Q(p)\,q = \bar{Q}(q)\,p$:

$$
Q(p) =
\begin{bmatrix}
p_0 & -p_1 & -p_2 & -p_3 \\
p_1 &  p_0 & -p_3 &  p_2 \\
p_2 &  p_3 &  p_0 & -p_1 \\
p_3 & -p_2 &  p_1 &  p_0
\end{bmatrix}
\qquad
\bar{Q}(q) =
\begin{bmatrix}
q_0 & -q_1 & -q_2 & -q_3 \\
q_1 &  q_0 &  q_3 & -q_2 \\
q_2 & -q_3 &  q_0 &  q_1 \\
q_3 &  q_2 & -q_1 &  q_0
\end{bmatrix}
$$

where:

- $Q(p)\,q$: the product expressed with $p$'s left-multiplication matrix
- $\bar{Q}(q)\,p$: the same product expressed with $q$'s right-multiplication matrix

---

### 3. Norm, Conjugate and Inverse

The norm/length of a quaternion is defined, just as for any complex number, as:

$$
\lVert q \rVert = \sqrt{q_0^2+q_1^2+q_2^2+q_3^2}
$$

All quaternions in the presented approach are of unitary length and are called **unit quaternions**. The complex conjugate switches the sign of the vector part:

$$
q^* = [\,q_0,\,-q_1,\,-q_2,\,-q_3\,]^T
$$

and the inverse is defined as for a complex number:

$$
q^{-1} = \frac{q^*}{\lVert q \rVert^2}
$$

where:

- $q^*$: conjugate of $q$
- $\lVert q \rVert^2$: squared norm of $q$

For a unit quaternion the inverse equals the conjugate, since $\lVert q \rVert = 1$.

---

### 4. Quaternion Time Derivative

The derivative of a quaternion has two forms depending on the frame in which the angular velocity $\omega$ is expressed. If $\omega$ is in the **fixed (world) frame**:

$$
\dot q_\omega(q,\omega) = \tfrac{1}{2}\, q \otimes [0,\omega]^T
= \tfrac{1}{2} Q(q)[0,\omega]^T
$$

If $\omega'$ is in the **body frame**:

$$
\dot q_{\omega'}(q,\omega') = \tfrac{1}{2}\, [0,\omega']^T \otimes q
= \tfrac{1}{2} \bar{Q}(q)[0,\omega']^T
$$

where:

- $q$: the attitude quaternion
- $\omega = [\omega_x, \omega_y, \omega_z]^T$: angular velocity in the fixed frame
- $\omega'$: angular velocity in the body frame
- $[0,\omega]^T$: the pure quaternion formed from $\omega$

These notations are provided with respect to the left-hand notation; converting to right-hand notation requires conjugating the $\omega$ quaternion. The body-frame form (eq. 7) is the one the paper combines with the rotational dynamics in eq. (18).

---

### 5. Vector Rotation and the DCM

A unit quaternion can be used as a rotation operator. The transformation requires **two** multiplications — the quaternion and its conjugate — forming the sandwich product that rotates the vector $v$ from the fixed frame into the body frame represented by $q$:

$$
w = q \otimes [0,v]^T \otimes q^*
$$

Replacing $v$ with the $x$, $y$ and $z$ axes expands this rotation column by column:

$$
\begin{aligned}
R_x(q) &= \begin{bmatrix} q_0^2+q_1^2-q_2^2-q_3^2 \\ 2(q_1q_2+q_0q_3) \\ 2(q_1q_3-q_0q_2)\end{bmatrix} \\[6pt]
R_y(q) &= \begin{bmatrix} 2(q_1q_2-q_0q_3) \\ q_0^2-q_1^2+q_2^2-q_3^2 \\ 2(q_2q_3+q_0q_1)\end{bmatrix} \\[6pt]
R_z(q) &= \begin{bmatrix} 2(q_1q_3+q_0q_2) \\ 2(q_2q_3-q_0q_1) \\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}
\end{aligned}
$$

Stacking the three columns gives the rotation (direction cosine) matrix that rotates a point in a fixed coordinate system:

$$
R(q) = \big[\,R_x(q) \;\; R_y(q) \;\; R_z(q)\,\big]
$$

When rotating a **coordinate system** instead of a point, the angle sign changes and the transpose arises:

$$
R(q) = \big[\,R_x(q)^T \;\; R_y(q)^T \;\; R_z(q)^T\,\big]^T
$$

where:

- $R_x(q), R_y(q), R_z(q)$: the images of the body axes under the sandwich product
- $R(q)$: the $3\times3$ direction cosine matrix, orthonormal with $\det R = +1$

In this project the DCM is implemented and tested (eq. 12 column form) but is **never used inside the plant or the controller** — it exists only to verify the rotation convention and for 3-D visualization.

---

### 6. Axis–Angle and Euler Conversions

The rotation can also be represented using a rotation vector, where $u$ is the unit rotation axis and $\alpha$ the angle of rotation:

$$
q = \cos(\alpha/2) + u\sin(\alpha/2)
$$

where:

- $u$: unit rotation axis
- $\alpha$: rotation angle

This form has a direct physical connection and is useful when creating an error or specifying a reference. For intuitive display, Euler angles convert to and from a quaternion in closed form:

$$
q = \begin{bmatrix}
\cos\frac\phi2\cos\frac\theta2\cos\frac\psi2+\sin\frac\phi2\sin\frac\theta2\sin\frac\psi2\\
\sin\frac\phi2\cos\frac\theta2\cos\frac\psi2-\cos\frac\phi2\sin\frac\theta2\sin\frac\psi2\\
\cos\frac\phi2\sin\frac\theta2\cos\frac\psi2+\sin\frac\phi2\cos\frac\theta2\sin\frac\psi2\\
\cos\frac\phi2\cos\frac\theta2\sin\frac\psi2-\sin\frac\phi2\sin\frac\theta2\cos\frac\psi2
\end{bmatrix}
$$

$$
\begin{bmatrix}\phi\\\theta\\\psi\end{bmatrix} =
\begin{bmatrix}
\operatorname{atan2}\!\big(2(q_0q_1+q_2q_3),\, q_0^2-q_1^2-q_2^2+q_3^2\big)\\
\arcsin\!\big(2(q_0q_2-q_3q_1)\big)\\
\operatorname{atan2}\!\big(2(q_0q_3+q_1q_2),\, q_0^2+q_1^2-q_2^2-q_3^2\big)
\end{bmatrix}
$$

where $\phi, \theta, \psi$ are the roll, pitch and yaw angles (aerospace ZYX convention).

Equation (16) is used **only** for plotting and logging in this implementation — the entire closed loop operates on $q$ directly, which is the whole point of the paper.

---

### 7. Rigid-Body Attitude Dynamics

For modeling the physics of the quadrotor, the Euler–Newton equations for the translational and rotational dynamics of a rigid body are used:

$$
\begin{bmatrix} F \\ \tau \end{bmatrix} =
\begin{bmatrix} m & 0 \\ 0 & I_{cm} \end{bmatrix}
\begin{bmatrix} a_{cm} \\ \dot\omega \end{bmatrix}
+ \begin{bmatrix} 0 \\ \omega \times (I_{cm}\omega) \end{bmatrix}
$$

where:

| Symbol | Parameter |
|---|---|
| $m$ | vehicle mass |
| $I_{cm}$ | inertia tensor about the center of mass |
| $F$ | total force |
| $\tau$ | total torque |
| $a_{cm}$ | center-of-mass acceleration |
| $\omega$ | angular velocity, $\omega = [\omega_x,\omega_y,\omega_z]^T$ |
| $\omega \times (I_{cm}\omega)$ | gyroscopic coupling term |

Combining the quaternion kinematics with the rotational half of eq. (17) gives the paper's attitude plant:

$$
\boxed{
\begin{aligned}
\dot q &= -\tfrac12\,[0,\omega]^T \otimes q \\
\dot\omega &= I_{cm}^{-1}\tau - I_{cm}^{-1}\big[\omega \times (I_{cm}\omega)\big]
\end{aligned}
}
$$

The leading **minus** sign on $\dot q$ is not a typo — it is the paper's own published convention, the opposite sign of the general body-frame form in eq. (7), and is preserved literally in the implementation. A dedicated test checks that one closed-loop integration step reduces attitude error, which would fail loudly if either eq. (18)'s or eq. (21)'s sign were wrong. One consequence worth stating plainly: because of this sign, a *positive* commanded $\omega$ about an axis *reduces* an existing positive attitude error on that axis — the opposite of the textbook $\dot q=+\tfrac12[0,\omega]\otimes q$ intuition.

The control-signal-to-torque relation is modelled as the identity, exactly as the paper simplifies it. The paper's simulation parameters are the Luleå University CAD-derived inertia:

$$
I_{xx}=I_{yy}=6.5\times10^{-4}\ \mathrm{kg\,m^2},\qquad
I_{zz}=1.2\times10^{-3}\ \mathrm{kg\,m^2}
$$

---

### 8. Nonlinear $P^2$ Attitude Controller

Given a reference quaternion $q_{ref}$ and a measured quaternion $q_m$, the error quaternion is obtained by multiplying the reference with the conjugate of the measured quaternion:

$$
q_{err} = q_{ref} \otimes q_m^{*}
$$

where:

- $q_{ref}$: desired attitude quaternion
- $q_m$: measured/estimated attitude quaternion
- $q_m^*$: conjugate of $q_m$

The vector part of $q_{err}$ directly connects to the **sine of half the rotation error** (from eq. 14) and provides the per-axis attitude error:

$$
\mathrm{Axis}_{err} = [\,q_{err,1},\,q_{err,2},\,q_{err,3}\,]^T
$$

Because a rotation of more than $\pi$ radians has a shorter path in the opposite direction, the sign of $\mathrm{Axis}_{err}$ is flipped whenever $q_{err,0}<0$ (the "shortest path" correction) — **except** in the 360° flip scenario, where this correction is deliberately disabled so the controller commits to the full rotation rather than reversing.

The control law is a derivative-free, non-linear proportional-squared feedback on the attitude and rate errors:

$$
\boxed{
\tau = -P_q\,\mathrm{Axis}_{err} - P_\omega\,\omega_m
}
$$

where:

- $\tau$: commanded control torque
- $P_q$: attitude-error proportional gain (outer loop)
- $\mathrm{Axis}_{err}$: vector part of the quaternion error
- $P_\omega$: angular-velocity damping gain (inner loop)
- $\omega_m$: measured angular velocity

with the paper's tuned gains $P_q=20$, $P_\omega=4$, and $\pm4\,\mathrm{N\,m}$ per-axis torque saturation.

$P_q$ — attitude proportional gain: determines how strongly the torque responds to the quaternion error.
Higher $P_q$ → the quadrotor rotates toward the reference more aggressively.
Too high → larger torque spikes at step onset and stronger amplification of measurement noise.

$P_\omega$ — rate damping gain: resists rotation and provides damping.
Higher $P_\omega$ → more damping, less overshoot.
Too high → the rate feedback over-damps the response and slows convergence.

No integral term is present or added: the plant's own double-integrator structure (torque → $\dot\omega$ → attitude) drives steady-state error to zero without one, and omitting the integrator adds no negative phase shift from the controller.

---

## Expected Results

- Exact reproduction of all paper parameters: $I_{xx}=I_{yy}=6.5\times10^{-4}$, $I_{zz}=1.2\times10^{-3}\ \mathrm{kg\,m^2}$, $P_q=20$, $P_\omega=4$, $\pm4$ N·m saturation, 0.1 noise amplitude.
- **Step response**: staggered 1 rad steps on $\phi,\theta,\psi$ with essentially no overshoot and settling within approximately 2 s per axis, torque briefly saturating at each step onset and returning to its linear region.
- **Sinusoidal tracking**: 0.5 rad, 1 rad/s sines on all axes tracked with a small phase lag of roughly 0.4–0.5 s, matching the paper's stated "about 0.5 second", with torque staying in its linear region during steady-state tracking.
- **360° flip**: the raw quaternion components $q_0,q_1$ remain smooth throughout the full rotation with no discontinuity — demonstrating the absence of any gimbal-lock artifact even though the *plotted* Euler angle $\phi$ necessarily wraps at $\pm\pi$.
- All 7 reproduction tests passing, including the guardrail test on the eq. (18) sign convention.

## Results and Comparison with the Base Paper

### 1. Parameter Verification

Every numeric parameter stated in the paper is reproduced exactly in the implementation:

| **Quantity** | **Paper Value** | **Implemented Value** | **Match** |
|---|---|---|---|
| $I_{xx}=I_{yy}$ | $6.5\times10^{-4}\ \mathrm{kg\,m^2}$ | $6.5\times10^{-4}$ | Exact |
| $I_{zz}$ | $1.2\times10^{-3}\ \mathrm{kg\,m^2}$ | $1.2\times10^{-3}$ | Exact |
| $P_q$ | $20$ | $20$ | Exact |
| $P_\omega$ | $4$ | $4$ | Exact |
| Torque saturation | $\pm4\ \mathrm{N\,m}$ | $\pm4\ \mathrm{N\,m}$ | Exact |
| Sine amplitude / frequency | $0.5$ rad, $1$ rad/s | $0.5$ rad, $1$ rad/s | Exact |
| Flip range | $0\to2\pi$ rad | $0\to2\pi$ rad | Exact |
| Noise amplitude | $0.1$ | $0.1$ | Exact |

### 2. Step Response (Paper Figs. 3–4)

A 1 rad step is commanded on $\phi$, $\theta$, $\psi$ in turn at staggered times over a 15 s run, under the full 0.1 measurement noise:

| **Metric** | **Paper (Fig. 3–4)** | **This Project (measured)** | **Verdict** |
|---|---|---|---|
| Overshoot | "very small overshoot" | $\le 0.009$ rad on all axes | Matches |
| Settling | "errors go quickly to zero" | 5% band settling in $\approx 1.2$ s per axis | Matches |
| Torque behaviour | saturates at step onset, returns to linear region | saturates briefly at each onset, then linear | Matches |

<p align="center">
  <img src="docs/figures/step_attitude.png" width="620" alt="Step response attitude tracking">
</p>

<p align="center">
  <img src="docs/figures/step_torque.png" width="620" alt="Step response torque">
</p>

### 3. Sinusoidal Tracking (Paper Figs. 5–6)

A $0.5$ rad, $1$ rad/s sine is tracked on all three axes with phase offsets so the torques are not in phase:

| **Metric** | **Paper (Fig. 5–6)** | **This Project (measured)** | **Verdict** |
|---|---|---|---|
| Phase lag | "about 0.5 second" | $0.37$–$0.39$ s (first-harmonic fit; linear theory predicts $0.3805$ s) | Close match |
| Tracking amplitude | good reference tracking | $0.5$ rad reference tracked at $0.46$–$0.47$ rad amplitude | Matches |
| Torque behaviour | no saturation during tracking | torque remains essentially in the linear region, with only brief noise-driven spikes | Matches |

<p align="center">
  <img src="docs/figures/sine_attitude.png" width="620" alt="Sine tracking attitude">
</p>

<p align="center">
  <img src="docs/figures/sine_torque.png" width="620" alt="Sine tracking torque">
</p>

### 4. 360° Flip (Paper Figs. 7–8)

With the shortest-path correction disabled, the reference ramps from $0$ to $2\pi$ rad about the body $x$-axis over 2 s:

| **Metric** | **Paper (Fig. 7–8)** | **This Project (measured)** | **Verdict** |
|---|---|---|---|
| Maneuver completion | flip executed without any problem | full $2\pi$ completed; final attitude $0.1^\circ$ from identity ($q_0\approx-1$, the double cover) | Matches |
| Quaternion smoothness | "no non-linearities nor singularities" | max per-ms jump in $q_0,q_1 \approx 0.0016$ — smooth throughout | Matches |
| Gimbal lock | none | none — the *plotted* Euler $\phi$ wraps at $\pm\pi$, an inherent property of `atan2`, not of the controller | Matches |

<p align="center">
  <img src="docs/figures/flip_attitude.png" width="620" alt="360-degree flip attitude">
</p>

<p align="center">
  <img src="docs/figures/flip_animation.gif" width="480" alt="Animated 3D playback of the 360-degree flip">
</p>

### 5. Test Suite

| **Suite** | **Tests** | **Status** |
|---|---|---|
| `tests/test_quaternion.py` | 7 (non-commutativity and matrix forms, conjugate identity, DCM orthonormality and Euler round-trip, rotation–DCM consistency, norm drift, zero-input fixed point, closed-loop sign consistency) | All pass |

**7 / 7 tests pass**, covering the algebraic properties of eqs. (1)–(16) and the negative-feedback consistency of the eq. (18) plant under the eq. (21) controller.

## Conclusion

This work reproduces every equation of Fresk and Nikolakopoulos's full quaternion attitude controller exactly, with both the plant and the controller operating entirely in quaternion space and Euler angles confined to plotting and logging. The reproduction is validated against the paper's own three benchmark scenarios — step, sinusoidal tracking and 360° flip — both quantitatively (parameters, overshoot, settling time, phase lag) and qualitatively (no saturation during steady tracking, no singularity through the full rotation), and every point at which the paper under-specifies a parameter is documented rather than silently chosen. The result is a running, testable demonstration of the paper's central claim: quaternion attitude control avoids gimbal lock entirely, even through a complete 360° rotation.
