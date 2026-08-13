# Mathematical establishment of rotation-prior-free calibration

This document derives the model implemented by
[planar_nav_rotation_calibrator.py](../../estimator/offline/tools/planar_nav_rotation_calibrator.py).
It is the mathematical companion to the
[algorithm methodology](fixed_translation_planar_multi_lidar_calibration.md).

The primary derivation is for the rotation-prior-free path: LiDAR translations
are constants, LiDAR rotations are estimated, the planar GNSS antenna lever
arm is a shared nuisance variable, and navigation uses only raw IMU and raw
dual-antenna GNSS. Signed motion axes reduce arbitrary $SO(3)$ initialization
to one planar yaw per LiDAR; known translations then make those yaw variables
observable. The older manifest-seeded path is retained only as a local-capture
comparison and shares the final rotation-only hand-eye equations.

## Mathematical overview

For $N$ LiDARs, the prior-free global state is

$$
\Theta_0=
[\beta_1,\ldots,\beta_N,g_x,g_y]^T,
$$

after one signed rotation axis has been recovered independently for every
LiDAR. The optional local refinement state is

$$
\Theta_{local}=
\{\delta_l\in\mathbb R^3\}_{l=1}^{N}
\cup\{g_x,g_y\}.
$$

The supplied translations $t_l$ satisfy

$$
\frac{\partial t_l}{\partial\Theta_0}=0,
\qquad
\frac{\partial t_l}{\partial\Theta_{local}}=0.
$$

The mathematical chain used by the implementation is

$$
\text{raw IMU/GNSS}
\rightarrow A^G_{ij}
\rightarrow A^V_{ij}(g),
\qquad
\text{identity ICP}
\rightarrow B^l_{ij},
$$

$$
B^l_{ij}=X_l^{-1}A^V_{ij}X_l
\rightarrow a_l=R_l^Tz_V
\rightarrow R_l(\beta_l)
\rightarrow (\beta_1,\ldots,\beta_N,g_x,g_y).
$$

## 1. State, frames, and transform convention

The notation $T_{AB}$ maps coordinates from frame $B$ into frame $A$.
For

$$
T=
\begin{bmatrix}
R&t\\
0&1
\end{bmatrix}
\in SE(3),
$$

$$
T^{-1}=
\begin{bmatrix}
R^T&-R^Tt\\
0&1
\end{bmatrix},
\qquad
T_1T_2=
\begin{bmatrix}
R_1R_2&R_1t_2+t_1\\
0&1
\end{bmatrix}.
$$

The frames are:

| Frame | Meaning |
|---|---|
| $W$ | local East-North-Up world fixed by the first valid GNSS position |
| $V$ | vehicle frame containing the supplied LiDAR translations |
| $G$ | GNSS antenna origin, with axes rotated parallel to $V$ |
| $L_l$ | coordinate frame of LiDAR $l$ |

For LiDARs $l=1,\ldots,N$, define

$$
X_l=T_{VL_l}
=
\begin{bmatrix}
R_l&t_l\\
0&1
\end{bmatrix}.
$$

Every supplied $t_l\in\mathbb R^3$ is constant. The local refinement state is

$$
\Theta=
\left\{\delta_l\in\mathbb R^3\right\}_{l=1}^{N}
\cup
\left\{\gamma=[g_x,g_y]^T\in\mathbb R^2\right\},
$$

where $\delta_l$ updates $R_l$, and

$$
g=P\gamma=[g_x,g_y,0]^T,
\qquad
P=
\begin{bmatrix}
1&0\\
0&1\\
0&0
\end{bmatrix},
$$

is the vehicle-to-GNSS-antenna lever arm. Thus

$$
T_{VG}=T(g)=
\begin{bmatrix}
I&g\\
0&1
\end{bmatrix}.
$$

The navigation trajectory, heading mounting yaw, LiDAR translations, time
offsets, and scan poses are not optimizer states.

## 2. Raw GNSS trajectory

### 2.1 WGS84 geodetic coordinates to local ENU

For latitude $\varphi$, longitude $\lambda$, altitude $h$, WGS84
semi-major axis $a=6378137\,\mathrm m$, and squared eccentricity
$e^2=6.69437999014\times10^{-3}$, define

$$
N(\varphi)=
\frac{a}{\sqrt{1-e^2\sin^2\varphi}}.
$$

The Earth-centred Earth-fixed position is

$$
p_E(\varphi,\lambda,h)=
\begin{bmatrix}
(N+h)\cos\varphi\cos\lambda\\
(N+h)\cos\varphi\sin\lambda\\
(N(1-e^2)+h)\sin\varphi
\end{bmatrix}.
$$

Let $(\varphi_0,\lambda_0,h_0)$ be the first usable fix and $p_{E0}$ its
ECEF position. The ECEF-to-ENU rotation is

$$
R_{WE}=
\begin{bmatrix}
-\sin\lambda_0&
\cos\lambda_0&
0\\
-\sin\varphi_0\cos\lambda_0&
-\sin\varphi_0\sin\lambda_0&
\cos\varphi_0\\
\cos\varphi_0\cos\lambda_0&
\cos\varphi_0\sin\lambda_0&
\sin\varphi_0
\end{bmatrix},
$$

so the raw antenna position in local ENU is

$$
p_G^W=R_{WE}(p_E-p_{E0}).
$$

### 2.2 Velocity-constrained interpolation

Each GNSS position $p_k$ has measured ENU velocity $v_k$. Between
$t_k$ and $t_{k+1}$, let

$$
s=\frac{t-t_k}{\Delta t},
\qquad
\Delta t=t_{k+1}-t_k.
$$

The cubic Hermite interpolation used at IMU and LiDAR timestamps is

$$
\begin{aligned}
p_G^W(t)={}&
h_{00}(s)p_k+h_{10}(s)\Delta t\,v_k\\
&+h_{01}(s)p_{k+1}+h_{11}(s)\Delta t\,v_{k+1},
\end{aligned}
$$

with

$$
\begin{aligned}
h_{00}(s)&=2s^3-3s^2+1,
&
h_{10}(s)&=s^3-2s^2+s,\\
h_{01}(s)&=-2s^3+3s^2,
&
h_{11}(s)&=s^3-s^2.
\end{aligned}
$$

## 3. Raw IMU and dual-heading attitude

### 3.1 Stationary initialization

For accelerometer $a_n$, gyroscope $\omega_n$, and interpolated planar
GNSS speed $v_n$, define

$$
\mathcal S=
\left\{
n:
\|v_n\|<0.10,\;
8.0<\|a_n\|<11.5
\right\}.
$$

The implementation initializes

$$
b_\omega=
\operatorname{median}_{n\in\mathcal S}(\omega_n),
\qquad
\bar g_B=
\operatorname{median}_{n\in\mathcal S}(a_n).
$$

Writing $\bar g_B=[g_x^B,g_y^B,g_z^B]^T$, the fixed planar roll and pitch are

$$
\phi=\operatorname{atan2}(g_y^B,g_z^B),
$$

$$
\theta=
\operatorname{atan2}
\left(
-g_x^B,
\sqrt{(g_y^B)^2+(g_z^B)^2}
\right).
$$

This assumes compatible IMU and vehicle roll/pitch axes.

### 3.2 High-rate yaw fused with dual-antenna heading

After stationary z-bias removal, trapezoidal integration gives

$$
\tilde\psi_n=
\tilde\psi_{n-1}
+
\frac{1}{2}
\left[
(\omega_{z,n-1}-b_z)+(\omega_{z,n}-b_z)
\right]
(t_n-t_{n-1}).
$$

At each GNSS heading timestamp $\tau_k$, a constant offset $c_0$ and
residual drift $c_1$ solve

$$
(c_0^*,c_1^*)=
\arg\min_{c_0,c_1}
\sum_k
\rho_{C_h}
\left(
\tilde\psi(\tau_k)
+c_0
+c_1(\tau_k-t_0)
-\psi_k^{GNSS}
\right),
$$

where $C_h=0.5^\circ$ and

$$
\rho_C(e)=
C^2\log
\left(
1+\frac{e^2}{C^2}
\right)
$$

is the scalar Cauchy loss. The dense heading-frame yaw is

$$
\psi_H(t)=
\tilde\psi(t)+c_0^*+c_1^*(t-t_0).
$$

Position and heading timestamps are handled independently.

### 3.3 Heading-baseline mounting yaw

For ENU velocity $v^W=[v_E,v_N,v_U]^T$, planar course is

$$
\chi_k=\operatorname{atan2}(v_{N,k},v_{E,k}).
$$

If surveyed heading-baseline mounting yaw $\alpha$ is unavailable, the
prototype estimates it on

$$
\mathcal C=
\left\{
k:
\sqrt{v_{E,k}^2+v_{N,k}^2}\ge2,\;
|\omega_z(\tau_k)-b_z|\le0.02
\right\}
$$

with a robust circular mean:

$$
\alpha=
\operatorname{Arg}
\left(
\sum_{k\in\mathcal C}
\exp
\left(
i\,\operatorname{wrap}
(\chi_k-\psi_H(\tau_k))
\right)
\right).
$$

This fallback assumes small sideslip on selected straight segments. A
surveyed $\alpha$ removes that assumption. The vehicle attitude is

$$
R_{WV}(t)=
R_z(\psi_H(t)+\alpha)
R_y(\theta)
R_x(\phi).
$$

## 4. Moving the navigation origin from antenna to vehicle

The constructed antenna pose is

$$
T_{WG}(k)=
\begin{bmatrix}
R_{WV}(k)&p_G^W(k)\\
0&1
\end{bmatrix}.
$$

Because $T_{WG}=T_{WV}T_{VG}$,

$$
T_{WV}(k)=T_{WG}(k)T(-g),
$$

and

$$
p_V^W(k)=p_G^W(k)-R_{WV}(k)g.
$$

For a time pair $(i,j)$, define antenna-origin motion

$$
A^G_{ij}
=
T_{WG}(i)^{-1}T_{WG}(j)
=
\begin{bmatrix}
Q_{ij}&d_{ij}\\
0&1
\end{bmatrix},
$$

where

$$
Q_{ij}=R_{WV}(i)^TR_{WV}(j),
$$

$$
d_{ij}=
R_{WV}(i)^T
\left(p_G^W(j)-p_G^W(i)\right).
$$

The vehicle-origin motion is

$$
\begin{aligned}
A^V_{ij}
&=
T_{WV}(i)^{-1}T_{WV}(j)\\
&=
T(g)A^G_{ij}T(-g)\\
&=
\begin{bmatrix}
Q_{ij}&d_{ij}+(I-Q_{ij})g\\
0&1
\end{bmatrix}.
\end{aligned}
$$

This conjugation is sign-sensitive. During turns, an offset antenna and the
vehicle origin do not have the same relative translation.

## 5. LiDAR hand-eye establishment

At time $k$, LiDAR $l$'s world pose is

$$
T_{WL_l}(k)=T_{WV}(k)X_l.
$$

Its physical motion from scan $j$ into scan $i$ is therefore

$$
\begin{aligned}
B^l_{ij}
&=
T_{WL_l}(i)^{-1}T_{WL_l}(j)\\
&=
X_l^{-1}A^V_{ij}X_l.
\end{aligned}
$$

For source point $p_m$ from scan $j$, target correspondence $q_m$ in
scan $i$, and target normal $n_m$, point-to-plane ICP measures

$$
\hat B^l_{ij}
=
\arg\min_{B\in SE(3)}
\sum_m
\left[
n_m^T(Bp_m-q_m)
\right]^2.
$$

In manifest mode, a hand-eye prediction from the input rotation initializes
ICP. In prior-free mode, every registration starts from identity; no
extrinsic rotation or translation is used to form correspondences. The final
measurement

$$
\hat B^l_{ij}
=
\begin{bmatrix}
S^l_{ij}&b^l_{ij}\\
0&1
\end{bmatrix}
$$

is fixed during calibration.

Expanding $X_l^{-1}A^V_{ij}X_l$ gives

$$
\widehat S^l_{ij}(R_l)=
R_l^TQ_{ij}R_l,
$$

$$
\begin{aligned}
\widehat b^l_{ij}(R_l,g)
&=
R_l^T
\left[
d_{ij}
+(I-Q_{ij})g
+(Q_{ij}-I)t_l
\right]\\
&=
R_l^T
\left[
d_{ij}
+(Q_{ij}-I)(t_l-g)
\right].
\end{aligned}
$$

The second equation is the central fixed-translation constraint. Both the
known LiDAR lever $t_l$ and unknown antenna lever $g$ affect turn-induced
translation, but only $R_l,g_x,g_y$ may change.

## 6. Rotation-prior-free initialization

### 6.1 Rotation-angle invariant and signed axes

Let $z_V$ be the unit planar rotation axis expressed in the vehicle frame. For
ideal planar motion,

$$
Q_{ij}=\operatorname{Exp}(\theta_{ij}[z_V]_\times).
$$

The LiDAR-frame rotation is

$$
S^l_{ij}=R_l^TQ_{ij}R_l
=\operatorname{Exp}
\left(\theta_{ij}[R_l^Tz_V]_\times\right).
$$

Conjugation preserves eigenvalues and hence rotation magnitude:

$$
\left\|\operatorname{Log}(S^l_{ij})\right\|=|\theta_{ij}|.
$$

This identity lets raw navigation reject an identity-seeded ICP result whose
angle is implausible without knowing $R_l$. For a nonzero accepted turn, raw
navigation also supplies the sign, so one signed axis sample is

$$
a^l_{ij}
=
\operatorname{sign}(\theta_{ij})
\frac{\operatorname{Log}(S^l_{ij})}
{\left\|\operatorname{Log}(S^l_{ij})\right\|}
\simeq R_l^Tz_V.
$$

The implementation uses a one-sample RANSAC over these axes, weights samples
by turn magnitude and ICP quality, rejects samples farther than four degrees,
and averages the inliers on the unit sphere. Navigation motions are treated
the same way to obtain the slightly tilted planar axis $z_V$ rather than
hard-coding $e_z$.

### 6.2 Axis alignment leaves one yaw

Let $a_l$ be the robust LiDAR-frame axis and let $R_{0l}$ be the minimum-angle
rotation satisfying

$$
R_{0l}a_l=z_V.
$$

Every rotation consistent with the recovered axes is then

$$
R_l(\beta_l)
=\operatorname{Exp}(\beta_l[z_V]_\times)R_{0l}.
$$

Thus arbitrary $SO(3)$ initialization has become one scalar yaw per LiDAR.

### 6.3 Fixed-translation Procrustes yaw

For a chosen planar antenna lever $g$, define the vehicle-frame translation
predicted by navigation and the known LiDAR lever:

$$
c^l_{ij}(g)
=d_{ij}+(I-Q_{ij})g+(Q_{ij}-I)t_l.
$$

The translational hand-eye equation requires

$$
c^l_{ij}(g)\simeq R_l(\beta_l)b^l_{ij}.
$$

Choose orthonormal vectors $u,v$ perpendicular to $z_V$ and project
$R_{0l}b^l_{ij}$ and $c^l_{ij}$ into that plane. For projected source
$s=[s_x,s_y]^T$, target $c=[c_x,c_y]^T$, and robust weight $w$, the global
two-dimensional Procrustes solution is

$$
\beta_l
=\operatorname{atan2}
\left(
\sum w(c_y s_x-c_x s_y),
\sum w(c_x s_x+c_y s_y)
\right).
$$

The implementation uses

$$
w_{lij}=
\frac{\sqrt{\max(0.05,f_{lij})}}
{\max(0.10,\sigma_{lij})},
$$

where $f_{lij}$ is ICP fitness and $\sigma_{lij}$ is ICP inlier RMSE.

This is a global solution on the circle for fixed $g$, not a small-angle
perturbation about a manifest rotation.

### 6.4 Joint multi-start yaw and antenna-lever solve

The prior-free initializer finally solves

$$
\Theta_0=[\beta_1,\ldots,\beta_N,g_x,g_y]^T
$$

with the robust translational objective

$$
\Theta_0^*
=\arg\min_{\Theta_0}
\sum_l\sum_{(i,j)\in\mathcal T_l}
\rho_C\left(
w_{lij}
\left[
R_l(\beta_l)b^l_{ij}-c^l_{ij}(g)
\right]
\right).
$$

Here $\rho_C$ is applied componentwise.

The implementation initializes each $\beta_l$ with the Procrustes expression
and tries planar lever seeds at zero, at every known LiDAR translation, and at
their median. LiDAR manifest rotations do not enter this state, its starts, or
its residual. The joint Hessian is $(N+2)\times(N+2)$.

## 7. Robust local alternating optimization

### 7.1 Rotation-only LiDAR block

Rotation uses a right perturbation of the selected initializer:

$$
R_l(\delta_l)=
R_{l,0}
\operatorname{Exp}
([\delta_l]_\times),
$$

where $[x]_\times y=x\times y$. Translation $t_l$ is absent from the
parameter block.

For pair $q=(i,j)$, define

$$
e^R_{lq}
=
\operatorname{Log}
\left(
(S^l_{ij})^T
\widehat S^l_{ij}
\right),
$$

$$
e^t_{lq}
=
\widehat b^l_{ij}-b^l_{ij}.
$$

ICP fitness $f_{lq}$ and inlier RMSE $\sigma_{lq}$ form

$$
w_{lq}
=
\frac{
\sqrt{\max(0.05,f_{lq})}
}{
\max(0.10,\sigma_{lq})
}.
$$

The implementation residual is

$$
r_{lq}
=
w_{lq}
\begin{bmatrix}
2e^R_{lq}\\
e^t_{lq}
\end{bmatrix}.
$$

The factor 2 is an engineering balance between radians and metres, not a
calibrated covariance. Conditional on $g$, LiDAR $l$ solves

$$
\delta_l^*
=
\arg\min_{\delta_l}
\frac{1}{2}
\sum_{q\in\mathcal T_l}
\sum_{c=1}^{6}
\log
\left(
1+r_{lq,c}(\delta_l)^2
\right),
$$

subject to the configured component bounds. $\mathcal T_l$ is the training
set; every fifth accepted constraint is held out.

### 7.2 Shared planar antenna-lever block

Conditional on all $R_l$, the lever solve uses weighted translation only:

$$
\gamma^*
=
\arg\min_{\gamma\in[-g_{\max},g_{\max}]^2}
\frac{1}{2}
\sum_l
\sum_{q\in\mathcal T_l}
\sum_{c=1}^{3}
\log
\left(
1+
[w_{lq}e^t_{lq,c}(P\gamma)]^2
\right).
$$

For fixed $R_l$, this residual is affine in $\gamma$, with unrobustified
Jacobian

$$
J^g_{lq}
=
\frac{\partial e^t_{lq}}{\partial\gamma}
=
R_l^T(I-Q_{ij})P.
$$

The algorithm alternates:

1. solve every $R_l$ independently for current $g$;
2. solve shared $[g_x,g_y]^T$ using all LiDARs; and
3. stop when the lever update is below $10^{-4}$ m or the iteration limit.

This is block-coordinate robust least squares, not a joint navigation factor
graph. Each rotation block is re-solved relative to the manifest-seeded or
prior-free initializer selected at the start of the run.

## 8. Planar observability

### 8.1 Ambiguity of rotation alone

Under ideal planar motion,

$$
Q_{ij}=R_z(\Delta\psi_{ij}).
$$

For arbitrary vehicle-frame yaw $\beta$, set

$$
R'_l=R_z(\beta)R_l.
$$

Then

$$
\begin{aligned}
(R'_l)^TQ_{ij}R'_l
&=
R_l^TR_z(-\beta)
R_z(\Delta\psi_{ij})
R_z(\beta)R_l\\
&=
R_l^TQ_{ij}R_l.
\end{aligned}
$$

The rotation equation identifies the vehicle z-axis direction in the LiDAR
frame, but not rotation about that axis.

### 8.2 Fixed translations remove the remaining ambiguity

The predicted LiDAR translation contains

$$
(Q_{ij}-I)(t_l-g).
$$

For a horizontal lever and nonzero yaw, its magnitude is

$$
2
\left\|
(t_l-g)_{xy}
\right\|
\left|
\sin\frac{\Delta\psi_{ij}}{2}
\right|.
$$

Changing the ambiguous vehicle-frame yaw of $R_l$ rotates this known signal
in the LiDAR frame. Long, diverse fixed LiDAR lever arms and motion pairs with
different yaw and displacement constrain the third rotational degree of
freedom. Optimizing $t_l$ would reintroduce rotation-translation
compensation.

Near a consistent solution, let

$$
u_{lq}
=
R_l^T
\left[
d_{ij}
+(I-Q_{ij})g
+(Q_{ij}-I)t_l
\right].
$$

For a small right perturbation
$E=\operatorname{Exp}([\delta]_\times)$,

$$
\widehat S(\delta)=
E^{-1}\widehat S(0)E,
\qquad
\widehat b(\delta)=
E^{-1}u_{lq}.
$$

At a small-residual solution,

$$
J^R_{lq}
\simeq
I-\widehat S(0)^T,
\qquad
J^t_{lq}
\simeq
[u_{lq}]_\times.
$$

The rotation term supplies two strong directions under yaw motion. The
translation term and changing $u_{lq}$ supply the remaining information.

### 8.3 Antenna height is unobservable

For planar $Q_{ij}=R_z(\Delta\psi)$,

$$
(I-Q_{ij})e_z=0.
$$

Hence the hand-eye translation has zero derivative with respect to $g_z$.
The implementation estimates only $g_x,g_y$ and fixes $g_z=0$. For
$\Delta\psi\ne0$, the 2D part satisfies

$$
\det(I_2-R_{2D})
=
4\sin^2\frac{\Delta\psi}{2}.
$$

Turning pairs make the planar lever observable; straight-only pairs do not.

### 8.4 Degenerate data

The system becomes weak or unobservable when:

- every $Q_{ij}$ is nearly identity;
- translation follows only one unchanged direction;
- LiDAR lever arms are short or geometrically redundant;
- ICP cannot measure consistent $B^l_{ij}$;
- turns do not excite GNSS and LiDAR lever effects; or
- course-based mounting yaw is invalid and no surveyed yaw is supplied.

## 9. Hessian, covariance, and acceptance

For each LiDAR, the solver returns robustified residual Jacobian $J_l$. The
reported Gauss-Newton information approximation is

$$
H_l=J_l^TJ_l.
$$

For residual dimension $m_l$,

$$
\widehat\sigma_l^2
=
\frac{r_l^Tr_l}{\max(1,m_l-3)},
\qquad
\Sigma_l
\approx
\widehat\sigma_l^2H_l^\dagger.
$$

The square roots of $\operatorname{diag}(\Sigma_l)$ are reported in degrees.
They are local diagnostics, not calibrated posterior uncertainty. The lever
information matrix is

$$
H_g=J_g^TJ_g\in\mathbb R^{2\times2}.
$$

The local rotation and lever solves require a positive minimum eigenvalue and
condition number below $10^6$. Solver success and the configured update bound
are also required. On the manifest-seeded path, each LiDAR must improve
held-out translation RMSE and must not worsen held-out rotation RMSE by more
than 5%; the shared lever solve must not materially regress held-out
translation RMSE.

For prior-free initialization, let $J_0$ be the Jacobian of the joint state
$\Theta_0$. Its information matrix

$$
H_0=J_0^TJ_0\in\mathbb R^{(N+2)\times(N+2)}
$$

must have minimum eigenvalue above $10^{-9}$ and condition number below
$10^8$. Every LiDAR also needs a stable signed axis and held-out translation
and rotation RMSE below 0.75 m and 2.5 degrees. A subsequent local
three-dimensional proposal is selected only if it reduces

$$
s_l=
\frac{\operatorname{RMSE}_{t,l}}{0.20\ \mathrm m}
+
\frac{\operatorname{RMSE}_{R,l}}{0.20^\circ}.
$$

Otherwise the valid global initializer is retained. For held-out set
$\mathcal H_l$,

$$
\operatorname{RMSE}_t
=
\sqrt{
\frac{1}{|\mathcal H_l|}
\sum_{q\in\mathcal H_l}
\|e^t_{lq}\|^2
},
$$

$$
\operatorname{RMSE}_R
=
\sqrt{
\frac{1}{|\mathcal H_l|}
\sum_{q\in\mathcal H_l}
\left(
\frac{180}{\pi}
\|e^R_{lq}\|
\right)^2
}.
$$

### 9.1 Optional rotation-only MLCC gate

The MLCC-style backend partitions transformed points into mixed-LiDAR planar
voxels. For voxel $v$, with point covariance $C_v$, its plane objective is the
smallest eigenvalue

$$
E_v=\lambda_{\min}(C_v),
\qquad
E=\sum_v E_v.
$$

When `optimize_extrinsic_translation: false`, Ceres receives every extrinsic
translation parameter block as constant. Only pose and extrinsic quaternion
blocks may update. The extrinsic observability test consequently uses the
rotation block

$$
H^{MLCC}_{R_l}\in\mathbb R^{3\times3}
$$

rather than the rank-deficient six-dimensional block containing three fixed
translation directions. Even a converged proposal is applied only when the
training and held-out plane objectives satisfy the configured acceptance
gates; rejection leaves both rotation and translation unchanged.

## 10. Map and relative extrinsics

For point $p^{L_l}$ in LiDAR $l$, the accepted ENU map point is

$$
\begin{aligned}
p^W
&=
R_{WV}
(R_lp^{L_l}+t_l)
+p_V^W\\
&=
p_G^W
+R_{WV}
(R_lp^{L_l}+t_l-g).
\end{aligned}
$$

Map position comes from raw GNSS antenna position corrected by $g$, while
every LiDAR translation remains fixed.

For reference LiDAR $r$, the relative extrinsic of LiDAR $l$ is

$$
T_{L_rL_l}
=
T_{VL_r}^{-1}T_{VL_l}
=
X_r^{-1}X_l.
$$

## 11. Gauge and scope summary

| Quantity | How it is fixed or estimated |
|---|---|
| ENU origin | first valid raw GNSS position |
| Global yaw | raw dual-antenna heading plus mounting yaw |
| Roll and pitch | stationary raw accelerometer, then fixed |
| High-rate yaw | bias-corrected raw gyro-z |
| Vehicle origin | raw antenna position minus $R_{WV}g$ |
| LiDAR translation $t_l$ | supplied constant; never optimized |
| LiDAR rotation $R_l$ | signed-axis plus global yaw, then guarded local refinement |
| GNSS lever $g_x,g_y$ | shared robust translation solve |
| GNSS lever $g_z$ | fixed to zero; planar-unobservable |

The map is globally referenced because raw GNSS fixes position and
dual-antenna heading fixes yaw. It is not a GNSS-denied SLAM proof: without
GNSS, global consistency requires loop closure and pose-graph or factor-graph
optimization. The global initializer does not use the MLCC-style backend, a
wheel model, a downstream navigation solution, or a LiDAR-derived vehicle
pose. MLCC is available only as a separately gated local refinement.

## 12. Code correspondence

| Mathematical block | Implementation function |
|---|---|
| WGS84 ECEF/ENU conversion | lla_to_enu |
| Stationary initialization, yaw fusion, Hermite trajectory | load_raw_navigation |
| $A^V=T(g)A^G T(-g)$ | motion_at_vehicle_origin |
| Motion-pair construction | select_pairs |
| Prior-free identity ICP | independent_motion_icp, collect_prior_free_constraints |
| Signed axes | robust_planar_axes, minimal_rotation_between |
| Procrustes and joint yaw/lever solve | planar_yaw_procrustes, estimate_prior_free_initialization |
| Manifest-seeded point-to-plane motion | multiscale_icp, collect_constraints |
| Rotation residual and solve | handeye_residual, calibrate_rotation |
| Shared planar $g$ solve | estimate_gnss_lever_arm |
| Rotation-only MLCC lock and Hessian | joint_backend.cpp: solveStage, fillHessianDiagnostics |
| ENU point transformation | NavigationTrajectory.poses, build_maps |
