# Full Quaternion-Based Attitude Control for a Quadrotor

### Equation Reference & Verification Document

> **Paper:** Emil Fresk & George Nikolakopoulos, *"Full Quaternion Based Attitude Control for a Quadrotor,"* European Control Conference (ECC) 2013, pp. 3864–3869.
>
> **Status:** All 21 equations verified against the PDF and confirmed implemented correctly in both the Python SITL (`quat_sitl/`) and the browser live simulator (`quadrotor_live.html`). Numerical cross-check: max $|\Delta| = 4.4 \times 10^{-11}$ between JavaScript and Python implementations.

---

## Notation & Conventions

| Symbol | Meaning |
|--------|---------|
| $q = [q_0,\; q_1,\; q_2,\; q_3]^T$ | Unit quaternion, **scalar-first** ($q_0$ = scalar part, $q_{1{:}3}$ = vector part) |
| $\otimes$ | Hamilton / Kronecker quaternion product (non-commutative) |
| $q^*$ | Quaternion conjugate |
| $q^{-1}$ | Quaternion inverse |
| $\omega = [\omega_x,\; \omega_y,\; \omega_z]^T$ | Angular velocity in the **body frame** |
| $I_{cm}$ | Diagonal inertia tensor of the rigid body |
| $\tau$ | Control torque (3-vector) |
| $P_q,\; P_\omega$ | Controller proportional gains |

**Left-hand vs. right-hand notation.** The paper states (Section II) that equations (6)–(7) are written in *left-hand notation*, and converting to *right-hand notation* requires conjugating the $\omega$-quaternion (negating its vector part). Equation (18) uses this right-hand (converted) form, which introduces a leading minus sign relative to equation (7). This is **not** an error — it is the correct conversion and is essential for closed-loop stability (see [Sign Convention](#sign-convention-eq-7-vs-eq-18) below).

---

## Section II — Quaternion Mathematics

### Eq. (1) — Quaternion representation (hypercomplex form)

$$q = q_0 + q_1\,i + q_2\,j + q_3\,k$$

### Eq. (2) — Column form & Kronecker product

$$q = \begin{bmatrix} q_0 \\ q_1 \\ q_2 \\ q_3 \end{bmatrix}$$

Multiplication of two quaternions $p \otimes q$:

$$p \otimes q = \begin{bmatrix} p_0 q_0 - p_1 q_1 - p_2 q_2 - p_3 q_3 \\ p_0 q_1 + p_1 q_0 + p_2 q_3 - p_3 q_2 \\ p_0 q_2 - p_1 q_3 + p_2 q_0 + p_3 q_1 \\ p_0 q_3 + p_1 q_2 - p_2 q_1 + p_3 q_0 \end{bmatrix}$$

**Left-multiplication matrix** $Q(p)$, such that $p \otimes q = Q(p)\,q$:

$$Q(p) = \begin{bmatrix} p_0 & -p_1 & -p_2 & -p_3 \\ p_1 & p_0 & -p_3 & p_2 \\ p_2 & p_3 & p_0 & -p_1 \\ p_3 & -p_2 & p_1 & p_0 \end{bmatrix}$$

**Right-multiplication matrix** $\bar{Q}(q)$, such that $p \otimes q = \bar{Q}(q)\,p$:

$$\bar{Q}(q) = \begin{bmatrix} q_0 & -q_1 & -q_2 & -q_3 \\ q_1 & q_0 & q_3 & -q_2 \\ q_2 & -q_3 & q_0 & q_1 \\ q_3 & q_2 & -q_1 & q_0 \end{bmatrix}$$

> **Code:** `quaternion.mul()`, `quaternion.Q()`, `quaternion.Q_bar()`

### Eq. (3) — Norm

$$\|q\| = \sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}$$

> **Code:** `quaternion.norm()` · All quaternions are assumed unitary ($\|q\| = 1$).

### Eq. (4) — Conjugate

$$q^* = \begin{bmatrix} q_0 & -q_1 & -q_2 & -q_3 \end{bmatrix}^T$$

> **Code:** `quaternion.conj()`

### Eq. (5) — Inverse

$$q^{-1} = \frac{q^*}{\|q\|^2}$$

For a unit quaternion, $q^{-1} = q^*$.

> **Code:** `quaternion.inv()`

### Eq. (6) — Quaternion derivative, fixed-frame $\omega$ (left-hand notation)

$$\dot{q}_\omega(q,\omega) = \frac{1}{2}\,q \otimes \begin{bmatrix} 0 \\ \omega \end{bmatrix} = \frac{1}{2}\,Q(q)\begin{bmatrix} 0 \\ \omega \end{bmatrix}$$

> **Code:** `quaternion.qdot_fixed()`

### Eq. (7) — Quaternion derivative, body-frame $\omega'$ (left-hand notation)

$$\dot{q}_{\omega'}(q,\omega') = \frac{1}{2}\begin{bmatrix} 0 \\ \omega' \end{bmatrix} \otimes q = \frac{1}{2}\,\bar{Q}(q)\begin{bmatrix} 0 \\ \omega' \end{bmatrix}$$

> **Code:** `quaternion.qdot_body()`
>
> **Note:** Both eqs. (6)–(7) use a **positive** sign. The paper notes these are in left-hand notation; converting to right-hand requires conjugating the $\omega$-quaternion.

### Eq. (8) — Vector rotation operator

$$w = q \otimes \begin{bmatrix} 0 \\ v \end{bmatrix} \otimes q^*$$

This rotates vector $v$ from the fixed frame to the body frame represented by $q$.

> **Code:** `quaternion.rotate()`

### Eq. (9) — DCM column $R_x(q)$

$$R_x(q) = q \otimes \begin{bmatrix} 0 \\ 1 \\ 0 \\ 0 \end{bmatrix} \otimes q^* = \begin{bmatrix} q_0^2 + q_1^2 - q_2^2 - q_3^2 \\ 2(q_1 q_2 + q_0 q_3) \\ 2(q_1 q_3 - q_0 q_2) \end{bmatrix}$$

### Eq. (10) — DCM column $R_y(q)$

$$R_y(q) = q \otimes \begin{bmatrix} 0 \\ 0 \\ 1 \\ 0 \end{bmatrix} \otimes q^* = \begin{bmatrix} 2(q_1 q_2 - q_0 q_3) \\ q_0^2 - q_1^2 + q_2^2 - q_3^2 \\ 2(q_2 q_3 + q_0 q_1) \end{bmatrix}$$

### Eq. (11) — DCM column $R_z(q)$

$$R_z(q) = q \otimes \begin{bmatrix} 0 \\ 0 \\ 0 \\ 1 \end{bmatrix} \otimes q^* = \begin{bmatrix} 2(q_1 q_3 + q_0 q_2) \\ 2(q_2 q_3 - q_0 q_1) \\ q_0^2 - q_1^2 - q_2^2 + q_3^2 \end{bmatrix}$$

### Eq. (12) — Direction Cosine Matrix (point rotation)

$$R(q) = \begin{bmatrix} R_x(q) & R_y(q) & R_z(q) \end{bmatrix}$$

$R_x, R_y, R_z$ are the **columns**. This matrix rotates a point from the body frame to the fixed frame: $v_{\text{world}} = R\,v_{\text{body}}$.

> **Code:** `quaternion.to_dcm()` · Verified: $R^T R = I$, $\det(R) = +1$.

### Eq. (13) — Direction Cosine Matrix (frame rotation)

$$R(q) = \begin{bmatrix} R_x(q)^T \\ R_y(q)^T \\ R_z(q)^T \end{bmatrix}$$

Rows instead of columns. This is the transpose of eq. (12) and rotates the coordinate *frame* rather than a point. Equivalently obtained by conjugating $q$ in eq. (8).

> **Code:** `to_dcm(q).T`

### Eq. (14) — Axis-angle to quaternion

$$q = \cos\!\left(\frac{\alpha}{2}\right) + \mathbf{u}\,\sin\!\left(\frac{\alpha}{2}\right)$$

where $\mathbf{u}$ is the unit rotation axis and $\alpha$ is the rotation angle.

> **Code:** `quaternion.from_axis_angle()`

### Eq. (15) — Euler angles to quaternion (aerospace ZYX: $\phi, \theta, \psi$)

$$q = \begin{bmatrix} \cos\frac{\phi}{2}\cos\frac{\theta}{2}\cos\frac{\psi}{2} + \sin\frac{\phi}{2}\sin\frac{\theta}{2}\sin\frac{\psi}{2} \\[4pt] \sin\frac{\phi}{2}\cos\frac{\theta}{2}\cos\frac{\psi}{2} - \cos\frac{\phi}{2}\sin\frac{\theta}{2}\sin\frac{\psi}{2} \\[4pt] \cos\frac{\phi}{2}\sin\frac{\theta}{2}\cos\frac{\psi}{2} + \sin\frac{\phi}{2}\cos\frac{\theta}{2}\sin\frac{\psi}{2} \\[4pt] \cos\frac{\phi}{2}\cos\frac{\theta}{2}\sin\frac{\psi}{2} - \sin\frac{\phi}{2}\sin\frac{\theta}{2}\cos\frac{\psi}{2} \end{bmatrix}$$

> **Code:** `quaternion.from_euler()`

### Eq. (16) — Quaternion to Euler angles

$$\begin{bmatrix} \phi \\ \theta \\ \psi \end{bmatrix} = \begin{bmatrix} \operatorname{atan2}\!\big(2(q_0 q_1 + q_2 q_3),\; q_0^2 - q_1^2 - q_2^2 + q_3^2\big) \\[4pt] \operatorname{asin}\!\big(2(q_0 q_2 - q_3 q_1)\big) \\[4pt] \operatorname{atan2}\!\big(2(q_0 q_3 + q_1 q_2),\; q_0^2 + q_1^2 - q_2^2 - q_3^2\big) \end{bmatrix}$$

> **Code:** `quaternion.to_euler()` · **Plotting/logging only** — never used inside the dynamics or control loop.

---

## Section III — Quaternion-Based Quadrotor Modeling

### Eq. (17) — Newton-Euler rigid-body dynamics

$$\begin{bmatrix} F \\ \tau \end{bmatrix} = \begin{bmatrix} m\,I & 0 \\ 0 & I_{cm} \end{bmatrix} \begin{bmatrix} a_{cm} \\ \dot{\omega} \end{bmatrix} + \begin{bmatrix} 0 \\ \omega \times (I_{cm}\,\omega) \end{bmatrix}$$

where $I_{cm} = \operatorname{diag}(I_{xx},\, I_{yy},\, I_{zz})$.

**Modeling assumptions** (paper Section III):
- Rigid, symmetrical structure
- Center of gravity coincides with body-frame origin
- Rigid propellers
- Gravity-bias throttle neglected (attitude-only)
- Only differential propeller forces affect rotation
- Control-signal to torque relation simplified to **identity**

### Eq. (18) — Full quaternion attitude dynamics (the plant)

Combining eq. (7) [converted to right-hand notation] with the rotational part of eq. (17):

$$\boxed{ \begin{cases} \dot{q} = -\dfrac{1}{2}\begin{bmatrix} 0 \\ \omega \end{bmatrix} \otimes q \\[8pt] \dot{\omega} = I_{cm}^{-1}\,\tau - I_{cm}^{-1}\big[\omega \times (I_{cm}\,\omega)\big] \end{cases} }$$

> **Code:** `dynamics.state_derivative()`
>
> **Critical sign note:** The leading **minus** in $\dot{q}$ is the paper's own — it arises from converting eq. (7)'s left-hand $+\frac{1}{2}[0;\omega']\otimes q$ to right-hand notation (conjugating $\omega$ flips the sign). The paper explicitly states eq. (18) combines "the right hand quaternion derivative from equation (7)." This sign is preserved literally in the code and is essential for the controller to produce negative feedback (see below).

---

## Section IV — Controller Synthesis

### Eq. (19) — Quaternion attitude error

$$q_{\text{err}} = q_{\text{ref}} \otimes q_m^*$$

where $q_{\text{ref}}$ is the reference (desired) quaternion and $q_m$ is the measured quaternion.

> **Code:** `controller.compute_torque()`, line `q_err = quat.mul(q_ref, quat.conj(q_m))`

### Eq. (20) — Axis error extraction

$$\text{Axis}_{\text{err}} = \begin{bmatrix} q_{\text{err},1} \\ q_{\text{err},2} \\ q_{\text{err},3} \end{bmatrix}$$

The vector part of $q_{\text{err}}$ directly connects to $\sin(\text{half-error-angle})$ via eq. (14). **Shortest-path rule:** if $q_{\text{err},0} < 0$ (meaning the desired orientation is more than $\pi$ away), the closest rotation is the conjugate direction, so the axis error is negated:

$$\text{if } q_{\text{err},0} < 0: \quad \text{Axis}_{\text{err}} \leftarrow -\text{Axis}_{\text{err}}$$

> **Code:** `if self.shortest_path and q_err[0] < 0.0: axis_err = -axis_err`
>
> The `shortest_path=False` option is used only for the 360° flip scenario, where the controller must commit to the full rotation.

### Eq. (21) — Nonlinear P² control law

$$\boxed{ \tau = -P_q \begin{bmatrix} q_{\text{err},1} \\ q_{\text{err},2} \\ q_{\text{err},3} \end{bmatrix} - P_\omega \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix} }$$

- **Outer loop** $P_q$: proportional gain on attitude error
- **Inner loop** $P_\omega$: proportional gain on angular rate (rate damping)

This controller is **derivative-free** and has **no integrator** — the double-integrator plant dynamics (eq. 18) drive the error to zero naturally, so no integral action (and no added phase lag) is needed.

> **Code:** `controller.compute_torque()`, return line `-self.gains.Pq * axis_err - self.gains.Pw * omega_m`

---

## Section V — Simulation Parameters

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Roll/pitch inertia | $I_{xx} = I_{yy}$ | $6.5 \times 10^{-4}\;\text{kg·m}^2$ | CAD model (paper Fig. 1, ref. [19]) |
| Yaw inertia | $I_{zz}$ | $1.2 \times 10^{-3}\;\text{kg·m}^2$ | CAD model |
| Attitude gain | $P_q$ | $20$ | Fine-tuned in simulation |
| Rate gain | $P_\omega$ | $4$ | Fine-tuned in simulation |
| Sensor noise | — | Additive, zero-mean, amplitude $0.1$ | All measurements |
| Torque saturation | $\tau_{\max}$ | $\pm 4\;\text{N·m}$ | All motors |

**Three evaluation scenarios** (paper Section V):

| Scenario | Description | Paper Figures |
|----------|-------------|---------------|
| **Step response** | 1 rad step on each axis at different time instants | Figs. 3–4 |
| **Sine tracking** | 0.5 rad amplitude, 1 rad/s sine, phase-shifted on $y,z$ | Figs. 5–6 |
| **360° flip** | Ramp from 0 to $2\pi$ radians about one axis | Figs. 7–8 |

---

## Sign Convention: Eq. (7) vs. Eq. (18)

The most subtle aspect of this paper is the sign relationship between eqs. (7) and (18):

| Equation | Form | Notation | Sign |
|----------|------|----------|------|
| Eq. (7) | $\dot{q} = +\frac{1}{2}[0;\omega']\otimes q$ | Left-hand | $+$ |
| Eq. (18) | $\dot{q} = -\frac{1}{2}[0;\omega]\otimes q$ | Right-hand | $-$ |

**Why the difference?** The paper states (Section II): *"these notations have been provided with respect to the left hand notation, and that for having them in the right hand notation the $\omega$ quaternion must be conjugated."* Conjugating $[0;\omega]$ gives $[0;-\omega]$, which flips the sign:

$$+\frac{1}{2}\begin{bmatrix}0\\\omega'\end{bmatrix}\otimes q \;\xrightarrow{\text{conjugate } \omega}\; +\frac{1}{2}\begin{bmatrix}0\\-\omega'\end{bmatrix}\otimes q = -\frac{1}{2}\begin{bmatrix}0\\\omega'\end{bmatrix}\otimes q$$

**Why it matters.** Linearizing the closed loop (eq. 18 + eq. 21) around identity gives a 2nd-order system:

$$I\,\ddot{e} + P_\omega\,\dot{e} + \frac{P_q}{2}\,e = 0$$

This is stable (overdamped, $\zeta \approx 25$ for the paper's gains) with two real poles at $\approx -2.5\;\text{rad/s}$ (slow, matches the few-second settling observed) and $\approx -P_\omega/I_{xx} \approx -6150\;\text{rad/s}$ (fast). The minus sign in eq. (18) is what makes the stiffness term $+P_q/(2I)$ **positive** (stable). Using the eq. (7) plus sign instead produces $-P_q/(2I)$ (negative stiffness → one positive real pole → **unstable**), which was verified numerically: the plus-sign variant diverges to $\pi$ rad error within 1 second.

---

## Numerical Integrator

The paper does not specify an ODE solver. This implementation uses **classic 4th-order Runge-Kutta (RK4)**:

$$\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}) \\
\mathbf{k}_2 &= f\!\left(\mathbf{x} + \tfrac{h}{2}\mathbf{k}_1\right) \\
\mathbf{k}_3 &= f\!\left(\mathbf{x} + \tfrac{h}{2}\mathbf{k}_2\right) \\
\mathbf{k}_4 &= f(\mathbf{x} + h\,\mathbf{k}_3) \\
\mathbf{x}_{n+1} &= \mathbf{x}_n + \frac{h}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right)
\end{aligned}$$

where $\mathbf{x} = [q_0, q_1, q_2, q_3, \omega_x, \omega_y, \omega_z]^T$ is the 7-element state vector, $f(\cdot) = $ eq. (18)'s `state_derivative`, and $h$ is the step size with the torque $\tau$ held constant (zero-order hold) across all four stages.

**Adaptive substepping.** The fast closed-loop eigenvalue $\lambda \approx P_\omega / I_{\min}$ requires the step size to satisfy $|\lambda \cdot h| \lesssim 0.5$ for RK4 stability. The simulator computes the number of substeps per control tick from the actual gains and inertia, ensuring numerical stability at any user-selected gain combination.

> **Code:** `integrator.rk4_step()` (Python), inline `rk4()` function (JavaScript live simulator)

---

## Verification Summary

| Check | Method | Result |
|-------|--------|--------|
| Eqs. (1)–(16): quaternion algebra | 100–200 random samples per equation, rebuilt independently from PDF text, compared to code at $10^{-12}$ tolerance | **24/24 PASS** |
| Eqs. (17)–(18): plant dynamics | Independent derivative computation, sign verification (eq. 18 ≠ eq. 7), inertia values | **PASS** |
| Eqs. (19)–(21): controller | Torque formula with shortest-path negation, gain values $P_q{=}20,\;P_\omega{=}4$, saturation $\pm 4$ | **PASS** |
| Closed-loop stability | 3 s step response converges to $< 0.001\;\text{rad}$ error | **PASS** |
| Plus-sign instability proof | Eq. (7) sign + eq. (21) → positive real pole at $+2.5\;\text{rad/s}$ → diverges to $\pi$ | **CONFIRMED** (proves minus sign is required) |
| JS ↔ Python cross-check | Identical single RK4 step, 7 state components | $\max|\Delta| = 4.4 \times 10^{-11}$ |

**Conclusion:** Every equation (1)–(21) is implemented exactly as printed in the paper. No methods from outside the paper are used in the dynamics or control loop. The only additions (RK4 integrator, adaptive substepping, noise distribution choice) are implementation infrastructure the paper does not specify.
