# Feasibility of rotation-prior-free planar multi-LiDAR calibration

Date: 2026-08-06

> **Implementation update (2026-08-13):** The proposed identity-ICP,
> signed-axis, and fixed-translation yaw initializer has now been implemented
> and run on all six AIV5 LiDARs. It succeeded without using manifest rotations
> and achieved 1.048-degree mean / 1.794-degree maximum relative rotation
> difference after scoring. See the
> [implementation result](aiv5_rotation_prior_free_results.md). Statements
> below that arbitrary-orientation recovery had not yet been demonstrated are
> retained as the conclusion of the earlier investigation, not the current
> repository status.

## Scope

This report investigates whether the AIV5 sequence can support multi-LiDAR
rotation calibration when each LiDAR translation in the vehicle frame is
precisely known but no usable LiDAR rotation initialization is available. It
also assesses whether a coarse vehicle trajectory fused from raw IMU, raw GNSS,
and future raw wheel measurements would improve the problem.

This is an observability and numerical-feasibility study. No algorithm was
implemented as part of this investigation. The excluded LiDAR-localizer
`vehicle_pose`, final INS result, and incomplete/final-result wheel text remain
excluded. In this report, "coarse vehicle pose" means a navigation trajectory
estimated directly from raw measurements, with uncertainty.

## Executive conclusion

The calibration is likely observable on this sequence, but an arbitrary
rotation cannot be expected to converge reliably by sending it directly to the
current ICP/MLCC backend. Precise translations, long sensor lever arms, and the
observed turning motion provide a strong geometric cue that can resolve the
remaining planar yaw ambiguity. The primary difficulty is global initialization
and reliable per-LiDAR motion estimation, especially for limited-field-of-view
LiDARs, rather than a fundamental lack of information.

A defensible architecture is:

1. fuse raw IMU and raw GNSS into an uncertain navigation trajectory;
2. estimate consecutive motion independently for each LiDAR;
3. recover the vehicle vertical in each LiDAR frame from rotation axes;
4. solve the remaining extrinsic yaw globally using known translations and
   navigation translation;
5. use MLCC/map consistency only as a local joint refinement backend; and
6. jointly or softly refine navigation corrections and LiDAR rotations rather
   than freezing a biased coarse trajectory.

Raw wheel velocity would improve short-term prediction, yaw-bias conditioning,
and robustness through GNSS gaps, but it is probably not required for this
particular sequence because it already contains continuous RTK-fixed position,
dual-antenna heading, high-rate IMU, and strong turns. Full Ackermann vehicle
kinematics are not required for a useful wheel factor. Raw, timestamped encoder
increments or forward speed are required; the current wheel text is not usable.

## What the existing 10-degree result proves

The existing experiment successfully recovered deterministic 10-degree
rotation perturbations to 0.632--1.130 degrees and produced a much more
consistent map. It proves a useful local capture range.

It does **not** prove rotation-prior-free recovery. The supplied coarse rotation
still initializes predicted LiDAR motion and multiscale ICP, while the rotation
solver estimates a bounded perturbation about that initial value. A reference
rotation was not used as a residual prior, but a useful initial orientation was
still part of correspondence formation. Removing the prior residual and
removing the global initialization are different problems.

## Motion model

Let the known vehicle-to-LiDAR translation and unknown rotation of LiDAR $i$ be

$$
X_i=T_{VL_i}=(R_i,t_i),
$$

where $t_i$ is fixed. Let the vehicle relative motion from time $k$ to $k+1$ be

$$
A_k=(Q_k,d_k),
$$

and let the same relative motion, estimated independently by LiDAR $i$, be

$$
B_{ik}=(S_{ik},b_{ik}).
$$

The hand-eye relation $A_kX_i=X_iB_{ik}$ expands to

$$
S_{ik}=R_i^TQ_kR_i,
$$

$$
R_i b_{ik}=d_k+(Q_k-I)t_i.
$$

If the navigation pose is located at a GNSS antenna or IMU rather than at the
declared vehicle origin, its surveyed or estimated lever arm must first be
included in $d_k$. Otherwise the omitted rotational lever-arm motion is
absorbed into the LiDAR rotations.

## Planar observability

For ideal planar motion,

$$
Q_k=R_z(\theta_k).
$$

The rotational equation determines

$$
a_i=R_i^Te_z,
$$

which is the vehicle vertical axis expressed in the LiDAR frame. It is invariant
to the replacement

$$
R_i\leftarrow R_z(\beta)R_i.
$$

Consequently, rotations alone cannot observe the LiDAR's yaw about the vehicle
vertical under planar motion.

The translation equation can resolve this one-dimensional ambiguity. Define

$$
c_k=d_k+(Q_k-I)t_i.
$$

After finding one rotation $R_{0i}$ satisfying $R_{0i}a_i=e_z$, every remaining
candidate has the form

$$
R_i=R_z(\beta_i)R_{0i}.
$$

The unknown $\beta_i$ can then be solved globally as a weighted two-dimensional
Procrustes problem:

$$
c_k\simeq R_z(\beta_i)R_{0i}b_{ik}.
$$

This is an important simplification: reliable planar motion converts an
arbitrary three-dimensional rotation search into rotation-axis estimation plus
a globally solvable one-dimensional yaw alignment.

### Why the known translations help

During a turn, $(Q_k-I)t_i$ is the displacement induced by rotating the known
sensor lever arm. Its magnitude is

$$
2\lVert t_{i,xy}\rVert\sin\left(\frac{|\theta_k|}{2}\right).
$$

The inspected AIV5 LiDAR lever arms are approximately 7.3--7.45 m. At the
selected-pair median yaw change of 5.47 degrees, the induced displacement is
approximately 0.69--0.71 m; the largest selected turns produce approximately
1.74--1.79 m. This is a strong yaw cue relative to normal RTK position noise.

Fixing translation also removes a major source of translation/rotation
compensation and eliminates the unobservable vertical-translation state that
commonly appears in planar calibration. It does not, however, make scan
registration convex or guarantee correct correspondences.

### Degenerate or fragile cases

The rotation estimate becomes unobservable or poorly conditioned when one or
more of the following hold:

- there is almost no turning, so the vehicle vertical cannot be recovered from
  relative rotations;
- motion is only repeated straight translation, which observes a direction but
  leaves rotation about that direction unconstrained;
- the LiDAR lies close to the turn centre and translational baselines are weak,
  leaving little information for the remaining yaw;
- single-LiDAR odometry is unreliable because of limited field of view,
  repetitive geometry, moving objects, motion distortion, or time offset;
- the navigation sensor-to-vehicle mounting is unknown, creating a common
  vehicle-frame orientation gauge; or
- a coarse navigation trajectory is treated as exact and its yaw or tilt bias
  is absorbed into all LiDAR rotations.

A common world-yaw gauge does not damage map consistency or relative
LiDAR-to-LiDAR rotations because it cancels in $R_i^TR_j$. It does matter when
the requested output is an absolute LiDAR rotation in a specifically defined
vehicle frame. Producing that output requires the IMU axes, dual-GNSS heading
baseline, or wheel forward direction to be related to the declared vehicle
frame.

## Numerically stable rotation-prior-free initialization

The current local optimizer should not be asked to solve arbitrary $SO(3)$
orientation and correspondence formation simultaneously. A more stable
initializer would perform the following conceptual stages:

1. **Independent LiDAR motion:** estimate $B_{ik}$ from consecutive scans of
   each LiDAR without using its vehicle extrinsic.
2. **Signed axis estimation:** estimate $a_i=R_i^Te_z$ from the rotation axes of
   $S_{ik}$; raw IMU/GNSS signed yaw increments resolve axis sign.
3. **Global planar yaw:** solve the one-dimensional Procrustes problem above,
   using robust weights derived from navigation and LiDAR-odometry covariance.
4. **Validation before mapping:** reject LiDARs or motion windows with weak
   singular values, inconsistent held-out hand-eye residuals, or unstable
   randomized solutions.
5. **Local refinement:** initialize scan-to-map association and the MLCC-style
   backend with this result, keep all LiDAR translations fixed, and refine only
   rotations and justified trajectory corrections.

If independent odometry fails for a limited-FOV LiDAR, plausible alternatives
are a bounded global rotation search scored by navigation-built map consistency,
or global feature registration followed by local GICP. Either alternative is a
coarse stage; MLCC remains the accuracy/refinement stage.

## Effect of an IMU/GNSS/wheel navigation trajectory

The helpful quantity is not a precomputed pose topic but a raw-sensor factor
graph or continuous-time trajectory with covariance.

### Raw IMU

Raw IMU supplies high-rate relative orientation, gravity, gyro-bias estimation,
and point-cloud deskewing. It strongly improves roll/pitch conditioning and the
timing of turns. It does not define vehicle yaw by itself, and the
IMU-to-vehicle mounting must be known or included as a calibration state.

### Raw GNSS

Raw GNSS position anchors global translation and long-term drift. A surveyed
dual-antenna baseline supplies direct heading; a single antenna supplies only
course-derived heading while moving and becomes weak at low speed. Antenna
lever-arm motion must be modeled during turns.

### Raw wheel measurements

Timestamped wheel speed or encoder increments add a short-term forward-velocity
measurement. Combined with a soft nonholonomic constraint, they can:

- stabilize motion between GNSS epochs and through short outages;
- improve gyro-yaw-bias estimation;
- define or reinforce the vehicle forward axis;
- improve scan-registration prediction; and
- reject navigation or LiDAR-motion outliers through redundancy.

Wheel data does not provide global position, absolute yaw, roll/pitch
excitation, or arbitrary LiDAR orientation by itself. Slip, wheel scale,
latency, unequal tyre radius, and aggressive turns can bias it. It should be a
robust factor with covariance and slip gating, not a hard constraint.

A full steering/Ackermann model is optional. A forward-speed measurement plus
soft lateral/vertical nonholonomic constraints is already useful. Steering
angle, wheelbase, or left/right encoder ticks can improve turn modeling but add
model and slip sensitivity.

### Fixed pose versus joint refinement

A fused coarse trajectory helps initialization and data association, but it
should not be frozen during final calibration. A small systematic navigation
yaw error is nearly indistinguishable from a common LiDAR yaw error if the pose
is treated as truth. The preferred final estimator retains navigation pose
corrections and IMU biases as states, uses raw GNSS and wheel measurements as
weighted factors, and estimates LiDAR rotations while holding their translations
exactly fixed.

## Dataset-specific assessment

The current sequence contains approximately:

| Quantity | Observed value |
|---|---:|
| Raw-GNSS path length | 152.89 m |
| Accumulated absolute yaw | 115.65 deg |
| Net yaw | 89.03 deg |
| Selected-pair median translation | 2.47 m |
| Selected-pair median absolute yaw | 5.47 deg |
| Selected-pair 90th-percentile absolute yaw | 12.74 deg |
| Maximum corrected gyro-z | 10.08 deg/s |

This is promising excitation for fixed-translation planar rotation calibration.
The local 10-degree solutions also had rotation-Hessian condition numbers of
approximately 15--31, which indicates acceptable local information near the
correct basin. Those Hessians do not establish uniqueness or convergence from
arbitrary initial rotations.

The largest remaining experimental uncertainty is whether every limited-FOV
LiDAR can supply sufficiently accurate independent motion estimates. The
360-degree LiDAR is expected to be easier. Time synchronization, per-point
deskewing, dynamic-object rejection, and scene geometry may determine success
more strongly than adding wheel data on this particular sequence.

## Recommended experiment before making a prior-free claim

The following study would separate observability from optimizer luck:

1. Generate per-LiDAR odometry without consulting any LiDAR extrinsic rotation.
2. Run the axis-plus-yaw initializer from uniformly sampled $SO(3)$ starting
   orientations, not only deterministic 10-degree perturbations.
3. Compare raw IMU+GNSS against raw IMU+GNSS+raw wheel measurements when valid
   wheel data becomes available.
4. Compare a frozen navigation trajectory with soft/joint navigation correction.
5. Keep reference extrinsics hidden until scoring.
6. Report the fraction entering a 5-degree MLCC capture basin, the fraction
   finishing below 1 degree, relative LiDAR-to-LiDAR rotation error, held-out
   hand-eye residuals, information singular values, and final map consistency.

Until that test succeeds, the supported statement is:

> The current method tolerates approximately 10 degrees of coarse rotation
> error locally on this AIV5 sequence. The sequence appears sufficiently
> excited for rotation-prior-free initialization, but arbitrary-orientation
> recovery has not yet been demonstrated.

## Related primary research

- Das et al., [Observability-Aware Online Multi-Lidar Extrinsic
  Calibration](https://arxiv.org/abs/2212.09579): starts from known CAD
  translation components, aligns independently estimated LiDAR trajectories to
  GNSS, and selects informative turning segments.
- Lv et al., [OA-LICalib](https://arxiv.org/abs/2205.03276): analyzes
  LiDAR--IMU degeneracy, selects informative segments, and suppresses
  unobservable updates through truncated singular-value decomposition.
- Yan et al., [An Extrinsic Calibration Method between LiDAR and GNSS/INS for
  Autonomous Driving](https://arxiv.org/abs/2209.07694): uses coarse-to-fine
  LiDAR/INS calibration specifically for ground vehicles under planar motion.
- Kulmer et al., [Multi-LiCa: A Motion and Targetless Multi LiDAR-to-LiDAR
  Calibration Framework](https://arxiv.org/abs/2501.11088): uses global feature
  registration before local GICP instead of relying on ICP to recover arbitrary
  transformations.
- Wang et al., [Joint Optimization-based Targetless Extrinsic Calibration for
  Multiple LiDARs and GNSS-Aided INS of Ground
  Vehicles](https://arxiv.org/abs/2507.08349): combines global rotation search
  with joint navigation-trajectory and extrinsic refinement for planar vehicles.
- LIWO, [LiDAR--IMU--Wheel Odometry](https://arxiv.org/abs/2302.14298): shows how
  wheel velocity improves state prediction and estimator robustness, while not
  replacing the need for extrinsic observability.
