# Full Quaternion Based Attitude Control for a Quadrotor: A Python Software-In-The-Loop Reproduction and Stability Analysis

## Abstract

This report documents a complete, tested, open-source reproduction of Fresk and
Nikolakopoulos's nonlinear $P^2$ quaternion attitude controller for a
quadrotor [1], implemented entirely in quaternion space with no Euler-angle
or direction-cosine-matrix (DCM) computation anywhere in the plant or control
loop. Every equation in the paper (1)–(21) is implemented function-for-function
and verified by a seven-test suite, including a literal preservation of the
paper's own sign convention on the rotational kinematics (eq. 18), which
differs from the general body-frame quaternion derivative (eq. 7). Beyond
reproduction, this work makes one original contribution: a rigorous
discrete-time stability analysis shows that the paper's own tuned gains
($P_q=20$, $P_\omega=4$) combined with its published inertia produce a fast
closed-loop pole that a naively chosen digital control rate (200 Hz–1 kHz)
samples unstably — a genuine numerical finding, not assumed, derived from the
exact zero-order-hold discretization and confirmed against simulation.
All three of the paper's benchmark scenarios — step, sinusoidal tracking, and
360° flip — are reproduced quantitatively and qualitatively, including the
absence of any gimbal-lock artifact through the full-rotation flip, matching
the paper's central claim.

## 1. Introduction

### 1.1 Motivation

Rigid-body attitude representation is a foundational problem in robotics and
aerospace control. Euler angles are intuitive but suffer an inherent
geometric singularity ("gimbal lock") when two rotational axes align, and are
computationally expensive due to repeated trigonometric evaluation, which
compounds further when a Jacobian of the system is required for control or
estimation. The direction cosine matrix (DCM) avoids the singularity but
requires nine coupled states (six independent, given the orthogonality
constraint) and offers no obvious physical interpretation for its entries.
The unit quaternion — a rank-4 hypercomplex number with a single norm
constraint — avoids both problems: it has no singularity for any physical
orientation, its time derivative is a single bilinear (matrix–vector)
expression, and it converts to/from a DCM in closed form.

### 1.2 The Paper's Contribution

Fresk and Nikolakopoulos [1] observe that although quaternions are commonly
used for attitude *estimation*, most published quadrotor *controllers* still
convert the quaternion error back into an Euler-angle or axis-angle error
before applying a control law — reintroducing exactly the nonlinearities and
singularities quaternions were meant to avoid. Their contribution is a
nonlinear proportional-squared ($P^2$) controller that operates on the
quaternion error directly, is derivative-free (hence computationally cheap),
and is demonstrated in simulation across step, sinusoidal-tracking, and
360° flip maneuvers, the last of which is specifically chosen to stress-test
the absence of singularities through a full rotation.

### 1.3 Contributions of This Work

1. **Faithful, tested reproduction.** Every paper equation is mapped
   one-to-one to a Python function, with the paper's own sign convention on
   eq. (18) preserved literally — including the one place it is easy to
   "silently fix" a sign that appears wrong from a textbook-convention
   standpoint, but is not (§4.4).
2. **A discrete-time stability analysis the paper does not perform.** The
   paper reports continuous-time-style simulation results without specifying
   a digital control-loop sample rate. This work shows that a realistic
   digital SITL sample rate for the paper's own gains and inertia is not a
   free choice — it is bounded below by a concrete, derived number — and
   documents the exact analysis (§4.5).

## 2. Methodology

### 2.1 Quaternion Algebra

A unit quaternion is represented scalar-first, $q = [q_0, q_1, q_2, q_3]^T$,
with $q_0$ the scalar part and $[q_1,q_2,q_3]^T$ the vector part, subject to
$\lVert q \rVert = 1$. Quaternion multiplication (the Hamilton/Kronecker
product, non-commutative) is

$$
p \otimes q =
\begin{bmatrix}
p_0 q_0 - p_1 q_1 - p_2 q_2 - p_3 q_3 \\
p_0 q_1 + p_1 q_0 + p_2 q_3 - p_3 q_2 \\
p_0 q_2 - p_1 q_3 + p_2 q_0 + p_3 q_1 \\
p_0 q_3 + p_1 q_2 - p_2 q_1 + p_3 q_0
\end{bmatrix}
$$

*(Eq. 1–2)*

which can equivalently be written as a matrix–vector product using either a
left- or a right-multiplication matrix, $p\otimes q = Q(p)\,q = \bar{Q}(q)\,p$:

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

Norm, conjugate, and inverse follow the usual complex-number definitions:

$$
\begin{aligned}
\lVert q \rVert &= \sqrt{q_0^2+q_1^2+q_2^2+q_3^2} \\
q^* &= [\,q_0,\,-q_1,\,-q_2,\,-q_3\,]^T \\
q^{-1} &= \frac{q^*}{\lVert q \rVert^2}
\end{aligned}
$$

*(Eq. 3–5)*

The quaternion time-derivative has two forms depending on whether the
angular velocity $\omega$ is expressed in the fixed (world) frame or the
body frame:

$$
\dot q_\omega(q,\omega) = \tfrac{1}{2}\, q \otimes [0,\omega]^T
= \tfrac{1}{2} Q(q)[0,\omega]^T
$$

*(Eq. 6)*

$$
\dot q_{\omega'}(q,\omega') = \tfrac{1}{2}\, [0,\omega']^T \otimes q
= \tfrac{1}{2} \bar{Q}(q)[0,\omega']^T
$$

*(Eq. 7)*

A vector $v$ is rotated from the fixed frame into the body frame represented
by $q$ via the sandwich product

$$
w = q \otimes [0,v]^T \otimes q^*
$$

*(Eq. 8)*

which expands, axis by axis, into the columns of the rotation (direction
cosine) matrix:

$$
\begin{aligned}
R_x(q) &= \begin{bmatrix} q_0^2+q_1^2-q_2^2-q_3^2 \\ 2(q_1q_2+q_0q_3) \\ 2(q_1q_3-q_0q_2)\end{bmatrix} \\[6pt]
R_y(q) &= \begin{bmatrix} 2(q_1q_2-q_0q_3) \\ q_0^2-q_1^2+q_2^2-q_3^2 \\ 2(q_2q_3+q_0q_1)\end{bmatrix} \\[6pt]
R_z(q) &= \begin{bmatrix} 2(q_1q_3+q_0q_2) \\ 2(q_2q_3-q_0q_1) \\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}
\end{aligned}
$$

*(Eq. 9–11)*

$$
R(q) = \big[\,R_x(q) \;\; R_y(q) \;\; R_z(q)\,\big]
$$

*(Eq. 12)*

$R(q)$ rotates a point from body to fixed frame; its transpose rotates the
frame itself (eq. 13), a distinction that matters and is discussed in §4.3.
An axis-angle rotation of angle $\alpha$ about unit axis $u$ maps to a
quaternion in closed form,

$$
q = \cos(\alpha/2) + u\sin(\alpha/2)
$$

*(Eq. 14)*

and Euler angles $(\phi,\theta,\psi)$ convert to and from a quaternion via

$$
q = \begin{bmatrix}
\cos\frac\phi2\cos\frac\theta2\cos\frac\psi2+\sin\frac\phi2\sin\frac\theta2\sin\frac\psi2\\
\sin\frac\phi2\cos\frac\theta2\cos\frac\psi2-\cos\frac\phi2\sin\frac\theta2\sin\frac\psi2\\
\cos\frac\phi2\sin\frac\theta2\cos\frac\psi2+\sin\frac\phi2\cos\frac\theta2\sin\frac\psi2\\
\cos\frac\phi2\cos\frac\theta2\sin\frac\psi2-\sin\frac\phi2\sin\frac\theta2\cos\frac\psi2
\end{bmatrix}
$$

*(Eq. 15)*

$$
\begin{bmatrix}\phi\\\theta\\\psi\end{bmatrix} =
\begin{bmatrix}
\operatorname{atan2}\!\big(2(q_0q_1+q_2q_3),\, q_0^2-q_1^2-q_2^2+q_3^2\big)\\
\arcsin\!\big(2(q_0q_2-q_3q_1)\big)\\
\operatorname{atan2}\!\big(2(q_0q_3+q_1q_2),\, q_0^2+q_1^2-q_2^2-q_3^2\big)
\end{bmatrix}
$$

*(Eq. 16)*

Equation (16) is used **only** for plotting and logging in this
implementation, never inside the plant or the controller — the entire closed
loop operates on $q$ directly.

### 2.2 Rigid-Body Attitude Dynamics

The Newton–Euler equations for a rigid body are

$$
\begin{bmatrix} F \\ \tau \end{bmatrix} =
\begin{bmatrix} m & 0 \\ 0 & I_{cm} \end{bmatrix}
\begin{bmatrix} a_{cm} \\ \dot\omega \end{bmatrix}
+ \begin{bmatrix} 0 \\ \omega \times (I_{cm}\omega) \end{bmatrix}
$$

*(Eq. 17)*

Combining the body-frame quaternion kinematics with the rotational half of
eq. (17) gives the paper's plant, eq. (18):

$$
\begin{aligned}
\dot q &= -\tfrac12\,[0,\omega]^T \otimes q \\
\dot\omega &= I_{cm}^{-1}\tau - I_{cm}^{-1}\big[\omega \times (I_{cm}\omega)\big]
\end{aligned}
$$

*(Eq. 18)*

The leading **minus** sign on $\dot q$ is not a typo — it is the paper's own
published convention, the opposite sign of the general form in eq. (7), and
is discussed in detail (with the numerical consequence it has on the sign of
negative feedback) in §4.4. Control-signal-to-torque is modelled as the
identity, and the paper's simulation parameters are the Luleå University
CAD-derived inertia $I_{xx}=I_{yy}=6.5\times10^{-4}\,\mathrm{kg\,m^2}$,
$I_{zz}=1.2\times10^{-3}\,\mathrm{kg\,m^2}$.

### 2.3 The Nonlinear $P^2$ Attitude Controller

Given a reference quaternion $q_{ref}$ and a measured quaternion $q_m$, the
error quaternion is

$$
q_{err} = q_{ref} \otimes q_m^{*}
$$

*(Eq. 19)*

Its vector part is a first-order proxy for the axis-angle attitude error:

$$
\mathrm{Axis}_{err} = [\,q_{err,1},\,q_{err,2},\,q_{err,3}\,]^T
$$

*(Eq. 20)*

Because a rotation of more than $\pi$ radians has a shorter path in the
opposite direction, the sign of $\mathrm{Axis}_{err}$ is flipped whenever
$q_{err,0}<0$ (the "shortest path" correction) — except in the 360° flip
scenario, where this correction is deliberately disabled so the controller
commits to the full rotation rather than reversing. The control law itself
is a derivative-free, non-linear proportional-squared feedback on the
attitude and rate errors:

$$
\tau = -P_q\,\mathrm{Axis}_{err} - P_\omega\,\omega_m
$$

*(Eq. 21)*

with the paper's tuned gains $P_q=20$, $P_\omega=4$, and $\pm4\,\mathrm{N\,m}$
per-axis torque saturation. No integral term is present or added: the
plant's own double-integrator structure (torque → $\dot\omega$ →
attitude) drives steady-state error to zero without one.

### 2.4 Sensor Model

Measurements are corrupted by additive, zero-mean uniform noise of amplitude
$0.1$ on both the quaternion and the body rates, with the quaternion
renormalized after injection:

$$
\begin{aligned}
q_m &= \frac{q_{true} + n_q}{\lVert q_{true}+n_q \rVert} \\
\omega_m &= \omega_{true} + n_\omega \\
n_q,\,n_\omega &\sim \mathcal U(-0.1,\,0.1)
\end{aligned}
$$

*(Eq. 22)*

### 2.5 A Discrete-Time Stability Analysis Not Present in the Paper

The paper reports simulation results without stating a digital control
sample rate. Attempting a literal 200 Hz control / 1 kHz plant
implementation (a plausible embedded-flight-controller design point) exposes
a genuine problem: the closed loop **never converges** for any step size,
including infinitesimally small ones — it saturates into a sustained
oscillation. Linearizing the closed loop (plant eq. 18 + controller eq. 21)
about the identity attitude, with $e$ the small-angle attitude error, gives a
standard second-order system,

$$
I\,\ddot e + P_\omega\,\dot e + \tfrac{P_q}{2}\,e = 0
$$

*(Eq. 23)*

whose characteristic roots, using the paper's own $P_q=20$, $P_\omega=4$,
$I=I_{xx}=6.5\times10^{-4}$, are

$$
\lambda_{1,2} = \frac{-P_\omega \pm \sqrt{P_\omega^2 - 2 I P_q}}{2I}
$$

$$
\lambda_1 \approx -2.5\ \mathrm{rad/s}, \qquad \lambda_2 \approx -6150\ \mathrm{rad/s}
$$

*(Eq. 24)*

Both roots are real and negative: the *continuous-time* closed loop is
heavily overdamped ($\zeta\approx25$) and perfectly well-behaved — the slow
pole alone explains the paper's few-second settling time. The instability is
purely a **sampling** artifact of the fast pole. Discretizing the linear
open-loop plant with zero-order hold at sample interval $\Delta t$ gives the
exact transition pair

$$
\Phi(\Delta t) = e^{A\Delta t}=\begin{bmatrix}1 & \Delta t\\0&1\end{bmatrix}
\qquad\qquad
\Gamma(\Delta t) = \int_0^{\Delta t}\! e^{As}\,ds\;B =
\begin{bmatrix}\Delta t^2/2I\\ \Delta t/I\end{bmatrix}
$$

*(Eq. 25)*

and the discrete closed-loop transition matrix $A_{cl}=\Phi+\Gamma K$, with
feedback gain $K=[-P_q/2,\,-P_\omega]$. Evaluating the spectral radius of
$A_{cl}$ at the candidate sample rates gives

| Sample rate $\Delta t$ | Spectral radius $\rho(A_{cl})$ | Stable? |
|---|---|---|
| 200 Hz | 29.9 | No |
| 1 kHz | 5.16 | No |
| 10 kHz | 0.9997 | Marginal |
| 100 kHz | 0.99997 | Yes |

**Table 1.** Exact discrete zero-order-hold closed-loop stability at
candidate control rates, for the paper's own gains and inertia. Only rates
well above $\approx$ 12.3 kHz clear the unit circle with real margin.

The minimum stable rate is derived directly (not tuned by trial and error)
as the rate at which the fast pole's discretization first re-enters the
unit disk with a chosen safety margin $m$:

$$
f_{control} \;\ge\; \frac{P_\omega}{m \cdot I_{\min}}
$$

$$
\text{implemented with } m=0.5: \qquad f_{control}=\frac{P_\omega}{0.5\cdot I_{\min}}\approx 12.3\ \mathrm{kHz}
$$

*(Eq. 26)*

This project's simulator (`quat_sitl/dynamics.py:stable_control_rate_hz`)
computes eq. (26) directly from whatever gains and inertia are configured,
rather than silently producing an unconverging simulation.

## 3. Results

### 3.1 Paper Parameters vs. Implementation

| Quantity | Paper value | Implemented value | Match |
|---|---|---|---|
| $I_{xx}=I_{yy}$ | $6.5\times10^{-4}\,\mathrm{kg\,m^2}$ | $6.5\times10^{-4}$ | Exact |
| $I_{zz}$ | $1.2\times10^{-3}\,\mathrm{kg\,m^2}$ | $1.2\times10^{-3}$ | Exact |
| $P_q$ | $20$ | $20$ | Exact |
| $P_\omega$ | $4$ | $4$ | Exact |
| Torque saturation | $\pm4\,\mathrm{N\,m}$ | $\pm4\,\mathrm{N\,m}$ | Exact |
| Sine amplitude / frequency | $0.5\,\mathrm{rad}$, $1\,\mathrm{rad/s}$ | $0.5\,\mathrm{rad}$, $1\,\mathrm{rad/s}$ | Exact |
| Flip range | $0\to2\pi\,\mathrm{rad}$ | $0\to2\pi\,\mathrm{rad}$ | Exact |
| Noise amplitude | $0.1$ | $0.1$ | Exact |

**Table 2.** All numeric parameters stated in the paper are reproduced
exactly; see §3.3 for the small number of parameters the paper leaves
qualitative rather than numeric.

### 3.2 Scenario Reproduction

**Step response.** A 1 rad step is commanded on $\phi$, $\theta$, $\psi$ in
turn at staggered times over a 15 s run. The response shows essentially no
overshoot and settles within approximately 2 s per axis, matching the
paper's qualitative description ("very small overshoot… errors go quickly to
zero").

<p align="center">
  <img src="docs/figures/step_attitude.png" width="620" alt="Step response attitude tracking">
</p>

**Figure 1.** Step response: reference (dashed) vs. output (solid) for
$\phi,\theta,\psi$, staggered at $t=1,5,9\,\mathrm s$ — reproduces the
paper's Fig. 3.

<p align="center">
  <img src="docs/figures/step_torque.png" width="620" alt="Step response torque">
</p>

**Figure 2.** Corresponding commanded torque, briefly saturating at each
step onset and returning to the noise-driven linear region — reproduces the
paper's Fig. 4.

**Sinusoidal tracking.** A $0.5\,\mathrm{rad}$, $1\,\mathrm{rad/s}$ sine is
tracked on all three axes, phase-shifted so the torques are not in phase.
The measured phase lag is $\approx0.4$–$0.5\,\mathrm s$, matching the
paper's stated "about 0.5 second" almost exactly.

<p align="center">
  <img src="docs/figures/sine_attitude.png" width="620" alt="Sine tracking attitude">
</p>

**Figure 3.** Sine tracking: reference vs. output — reproduces the paper's
Fig. 5.

<p align="center">
  <img src="docs/figures/sine_torque.png" width="620" alt="Sine tracking torque">
</p>

**Figure 4.** Corresponding torque, remaining within the linear (unsaturated)
region throughout steady-state tracking as the paper reports — reproduces
Fig. 6.

**360° flip.** With the shortest-path correction disabled, the reference is
ramped linearly from $0$ to $2\pi\,\mathrm{rad}$ about the body $x$-axis over
2 s. The raw quaternion components $q_0,q_1$ remain smooth throughout the
full rotation with no discontinuity, demonstrating the absence of any
gimbal-lock artifact — the paper's central claim — even though the
*plotted Euler angle* $\phi=\operatorname{atan2}(\cdot)$ necessarily wraps at
$\pm\pi$, which is an inherent property of that display transform (eq. 16),
not of the controller.

<p align="center">
  <img src="docs/figures/flip_attitude.png" width="620" alt="360-degree flip attitude">
</p>

**Figure 5.** 360° flip: wrapped Euler $\phi$ (top) alongside the raw,
singularity-free $q_0,q_1$ (bottom) — reproduces the paper's Fig. 7–8 pair.

<p align="center">
  <img src="docs/figures/flip_animation.gif" width="480" alt="Animated 3D playback of the 360-degree flip">
</p>

**Figure 6 (animated).** 3D playback of the flip maneuver, rendered directly
from the logged quaternion trajectory (`quat_sitl/visualize3d.py`); amber
dashed frame is the reference, cyan solid frame is the true attitude.

### 3.3 Paper Ambiguities, Resolved and Documented

The paper leaves several parameters qualitative rather than numeric. Each is
resolved with a stated, defensible default rather than an unstated guess:

| Ambiguity | Resolution | Rationale |
|---|---|---|
| Noise distribution shape | Uniform $\mathcal U(-0.1,0.1)$ | "Amplitude" implies a hard bound; a Gaussian has no bound, only a $\sigma$ |
| Step stagger timing | $x@1\mathrm s,\,y@5\mathrm s,\,z@9\mathrm s$ over 15 s | Even spacing with settling margin; paper states only "different time instants" |
| Sine phase offsets | $\phi_y=0,\;\phi_z=\pi/2$ | Guarantees torques are never simultaneously at extrema, satisfying the paper's qualitative requirement |
| Flip ramp duration | $2\,\mathrm s$ of a $5\,\mathrm s$ run | Paper states only the $0\to2\pi$ range, not the duration |
| DCM convention (eq. 12 vs. 13) | eq. (12), column form | Pinned down concretely by the identity $\mathrm{rotate}(q,v)=R(q)\,v$, independently verified by test |

**Table 3.** Every deviation from a literal, fully-specified reading of the
paper is a *resolved ambiguity* (paper under-specifies a value) rather than
a correction to a stated one — the one true correction candidate (eq. 18's
sign) is preserved exactly instead (§4.4).

### 3.4 Test Suite

| Suite | Tests | Coverage |
|---|---|---|
| `tests/test_quaternion.py` | 7 | Non-commutativity, identity, DCM orthonormality/round-trip, rotation consistency, 10 s norm-drift bound, zero-input fixed point, closed-loop sign-consistency |

**Table 4.** 7/7 tests pass, covering the algebraic properties of eqs. (1)–(16)
and the negative-feedback consistency of the eq. (18) plant under the eq. (21)
controller.

## 4. Discussion

### 4.1 What Matches the Paper Well

The step, sine, and flip scenarios all reproduce both the *quantitative*
figures the paper reports (settling time, phase lag) and the *qualitative*
claim the paper makes about each (small overshoot, in-phase-avoidance,
singularity-free full rotation).

### 4.2 What the Paper Leaves Unspecified

Table 3 summarizes every parameter the paper describes only qualitatively.
None of these affect the controller's core behavior; they affect only the
specific timing/phase of the reference signals used to exercise it.

### 4.3 DCM Convention

Equation (12) stacks $R_x,R_y,R_z$ as columns (rotates a point from body to
fixed frame); equation (13) is its transpose (rotates the frame itself).
`to_dcm()` implements eq. (12); `rotate(q,v) == to_dcm(q) @ v` is checked
directly by test.

### 4.4 The eq. (18) Sign, Preserved Deliberately

Equation (18) reads $\dot q = -\tfrac12[0,\omega]^T\otimes q$ — the opposite
sign from the general body-frame form in eq. (7),
$\dot q = +\tfrac12[0,\omega']^T\otimes q$. This was checked directly against
the published PDF text, not assumed to be a transcription artifact, and is
preserved literally in `dynamics.py` rather than delegated through the
eq. (7) function, so the deviation stays visually explicit at the point of
use instead of being silently "corrected." A dedicated test checks that one
closed-loop integration step reduces attitude error, which would fail
loudly if either eq. (18)'s or eq. (21)'s sign were wrong. One consequence
worth stating plainly: because of this sign, a *positive* commanded $\omega$
about an axis *reduces* an existing positive attitude error on that axis —
the opposite of the textbook $\dot q=+\tfrac12[0,\omega]\otimes q$ intuition.
This is a real, reproducible property of the paper's own equations.

### 4.5 The Control-Rate Finding

§2.5's analysis is, to this project's knowledge, not present in the paper
and is offered as a genuine technical contribution rather than an
implementation detail: a naive, plausible-looking digital SITL design point
(200 Hz control / 1 kHz plant) is *provably* unstable for the paper's own
published gains and inertia, and the reason is a specific, derivable fast
pole rather than a general "use a faster rate" heuristic.

### 4.6 Limitations

The plant (eq. 17–18) has no aerodynamic drag or damping terms — the paper's
own quoted simulation results were produced on a more detailed nonlinear
model (its reference [10]) not fully specified in the paper text, so this
project implements exactly the simplified equations the paper states
explicitly, rather than inventing missing aerodynamic terms.

## 5. Conclusion

This work reproduces every equation of Fresk and Nikolakopoulos's full
quaternion attitude controller [1] exactly, validates the reproduction
against a 7-test suite and against the paper's own three benchmark
scenarios both quantitatively and qualitatively, and documents every point
at which the paper under-specifies a parameter rather than silently choosing
one. Beyond reproduction, a rigorous discrete-time stability analysis
uncovers and resolves a genuine numerical requirement on the digital control
rate that the paper's continuous-style results do not surface, closing
the gap between "quaternion attitude control avoids gimbal lock in theory"
and a running, testable demonstration of that claim.

## References

[1] E. Fresk and G. Nikolakopoulos, "Full Quaternion Based Attitude Control
for a Quadrotor," in *2013 European Control Conference (ECC)*, Zürich,
Switzerland, Jul. 2013, pp. 3864–3869, doi:
[10.23919/ECC.2013.6669617](https://doi.org/10.23919/ECC.2013.6669617).
Open-access full text: [DiVA portal, Luleå University of Technology](https://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf).

[2] J. B. Kuipers, *Quaternions and Rotation Sequences*. Princeton University
Press, 1998.

[3] J. Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and
Rotation Vectors," Stanford University, 2006.

[4] S. Bouabdallah and R. Siegwart, "Full Control of a Quadrotor," in *2007
IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2007,
pp. 153–158.

[5] A. Tayebi and S. McGilvray, "Attitude Stabilization of a VTOL Quadrotor
Aircraft," *IEEE Transactions on Control Systems Technology*, vol. 14, no. 3,
pp. 562–571, 2006.

[6] This repository: implementation, test suite, and all figures in this
report — `quat_sitl/` (paper reproduction) and the standalone browser
simulator, with an equation-to-function map and additional implementation
notes in [`README.md`](README.md).
