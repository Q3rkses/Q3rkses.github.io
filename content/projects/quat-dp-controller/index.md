---
title: "Adaptive Backstepping DP Controller — Quaternion Formulation"
date: 2026-04-05
draft: false
summary: "Adaptive backstepping controller for dynamic positioning of an AUV, using a quaternion error-state formulation to avoid gimbal lock."
tags: ["control-systems", "robotics", "ROS2", "quaternions", "backstepping", "adaptive-control"]
---

{{< katex >}}

<pre style="font-size: 0.45em; line-height: 1.2; letter-spacing: 0.05em; overflow-x: auto; text-align: center;">
 ██████╗ ██╗   ██╗ █████╗ ████████╗███████╗██████╗ ███╗   ██╗██╗ ██████╗ ███╗   ██╗    ██████╗ ██████╗ 
██╔═══██╗██║   ██║██╔══██╗╚══██╔══╝██╔════╝██╔══██╗████╗  ██║██║██╔═══██╗████╗  ██║    ██╔══██╗██╔══██╗
██║   ██║██║   ██║███████║   ██║   █████╗  ██████╔╝██╔██╗ ██║██║██║   ██║██╔██╗ ██║    ██║  ██║██████╔╝
██║▄▄ ██║██║   ██║██╔══██║   ██║   ██╔══╝  ██╔══██╗██║╚██╗██║██║██║   ██║██║╚██╗██║    ██║  ██║██╔═══╝ 
╚██████╔╝╚██████╔╝██║  ██║   ██║   ███████╗██║  ██║██║ ╚████║██║╚██████╔╝██║ ╚████║    ██████╔╝██║     
 ╚══▀▀═╝  ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝    ╚═════╝ ╚═╝     
                                                                                                       
</pre>

<p style="text-align: center; opacity: 0.6; font-style: italic; margin-top: -0.5em;">
Avoids gimbal lock, has superior numerical stability to euler angles, and utilizes error state formulation to aboid 4x3 noninvertible transformation matrix.
</p>

---

## Overview

This is an **adaptive backstepping** controller used for dynamic positioning (DP) of an AUV. The core idea is to replace the standard Euler angle kinematics with a **quaternion error-state formulation**, which eliminates gimbal lock and gives a kinematic Jacobian that is invertible everywhere except at a full 180° error. Which when utilizing SSA is a  scenario that simply doesn't occur in practice.
The controller was implemented in ROS2 as part of [Vortex NTNU's](https://www.vortexntnu.no/) AUV stack.

---

## Conventions

The proof uses the notation below. Note that in the **ROS2 implementation**, the names differ from the mathematical derivation:

| Math | Code | Meaning |
|---|---|---|
| $J_e(\eta)$ | `L` | Error-state kinematic Jacobian $\in \mathbb{R}^{6 \times 6}$ |
| $T_e(q_e)$ | `Q` | Quaternion-to-angular-velocity transformation matrix |

All other notation follows Fossen (2021). The unit quaternion is written $q = \begin{bmatrix} \eta \\ \varepsilon \end{bmatrix}$ with $\eta \in \mathbb{R}$, $\varepsilon \in \mathbb{R}^3$, and $\|q\|^2 = 1$.

---

## AUV Dynamics

The AUV is governed by two coupled differential equations. The **kinematic equation** maps body-frame velocities to NED-frame rates:

$$\dot{\eta} = J_q(\eta)\nu \tag{A}$$

and the **dynamic equation** (Newton-Euler in body frame):

$$M\dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) + g_0 = \tau + \tau_{\text{dist}} \tag{B}$$

where the full state is $\eta = \begin{bmatrix} p^n \\ q \end{bmatrix}$ with $p^n = [x_n, y_n, z_n]^\top$ the NED position, and $\nu = \begin{bmatrix} v_{nb}^b \\ \omega_{nb}^b \end{bmatrix}$ the body-frame linear and angular velocities.

The standard quaternion kinematic matrix is:

$$J_q(\eta) = \begin{bmatrix} R(q_b^n) & 0 \\ 0 & T(q_b^n) \end{bmatrix}$$

where $R(q_b^n) \in SO(3)$ is the rotation matrix from NED to body, and $T(q_b^n)$ is derived using the quaternion derivative identity:

$$\dot{q} = \frac{1}{2}\begin{bmatrix} -\varepsilon^\top \\ \eta I_3 + S(\varepsilon) \end{bmatrix} \omega_{nb}^b = \frac{1}{2} q_b^n \otimes \begin{bmatrix} 0 \\ \omega_{nb}^b \end{bmatrix}$$

**The problem**: $J_q(\eta) \in \mathbb{R}^{7 \times 6}$ is non-square and therefore not invertible, and in our case, utilizing backstepping, we will later require the inverse of J.

---

## Error-State Quaternion Formulation

Instead of working with $\eta$ directly, we define the **quaternion error**:

$$\delta q = q_d^* \otimes q, \qquad e_q = \begin{bmatrix} \delta\phi \\ \delta\theta \\ \delta\psi \end{bmatrix}$$

where $q^* = q^{-1}$ for unit quaternions. The identity quaternion is $q_I = \begin{bmatrix} 1 \\ 0 \\ 0 \\ 0 \end{bmatrix}$, so $\delta q \to q_I$ as $q \to q_d$.

From the angle-axis representation $q = \begin{bmatrix} \cos(\alpha/2) \\ \hat{n}\sin(\alpha/2) \end{bmatrix}$, for small $\alpha$ we get $\varepsilon \approx \hat{n}\frac{\alpha}{2}$, so:

$$e_q \approx 2\delta\varepsilon \quad \text{for small angles } |\delta\theta| \lesssim 25°$$

This is significant: we can now use a **square** $6 \times 6$ Jacobian. Defining $T_e(q_e) = \eta_e I_3 + S(\varepsilon_e)$, we get:

$$\dot{e}_q = T_e(q_e)\,\omega_{nb}^b$$

and equations (A) and (B) become:

$$
\begin{align}
\dot{\eta} &= J_e(\eta)\nu \tag{A*} \\
M\dot{\nu} + C(\nu)\nu + D(\nu)\nu &= \tau + \tau_{\text{dist}} \tag{B*}
\end{align}
$$

with the **error-state Jacobian**:

$$J_e(\eta) = \begin{bmatrix} R(q_b^n) & 0 \\ 0 & T_e(q_e) \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

$J_e$ is smooth, differentiable, and **invertible for $|\delta\theta| < 180°$** — exactly what backstepping needs.

> We chose $\delta q = q_d^* \otimes q$ (left multiplication by conjugate) rather than $q \otimes q_d^*$, because local perturbations expressed in the body frame are more intuitive.

---

## Backstepping Design

### Step 1 — Position & Attitude Error

Define the tracking error (using the local perturbation formula for the quaternion part):

$$z_1 = \eta - \eta_d \doteq \begin{bmatrix} x - x_d \\ y - y_d \\ z - z_d \\ q_d^* \otimes q \end{bmatrix}$$

Attempt CLF: $V_1(z_1) = \frac{1}{2} z_1^\top z_1$

$$\dot{V}_1 = z_1^\top \dot{z}_1 = z_1^\top J_e(\eta)\nu$$

Treat $\nu$ as a virtual input. Choose the **virtual control law**:

$$\alpha = -J_e(q_e)^{-1} K_1 z_1, \quad K_1 > 0$$

so that $\dot{V}_1 = -z_1^\top K_1 z_1 + z_1^\top J_e(q_e) z_2$ with $z_2 = \nu - \alpha$.

### Step 2 — Velocity Error

Let $V_2 = \frac{1}{2} z_2^\top M z_2$. From (B\*) and using $M = M^\top > 0$, $\dot{M} = 0$:

$$\dot{V}_2 = z_2^\top M(\dot{\nu} - \dot{\alpha}) = z_2^\top\bigl(\tau - C(\nu)\nu - D(\nu)\nu - M\dot{\alpha}\bigr)$$

The compound CLF $V_c = V_1 + V_2$ gives, if all terms were known:

$$
\begin{aligned}
\tau_{\text{controller}} &= -J_e(q_e) z_1 - K_2 z_2 + M\dot{\alpha} + C(\nu)\nu \\
\Rightarrow \dot{V}_c &= -z_1^\top K_1 z_1 - z_2^\top K_2 z_2 < 0 \quad \forall z \neq 0
\end{aligned}
$$

### Adaptive Extension

We do not know the true $D(\nu)$ or $\tau_{\text{dist}}$. Write $D(\nu)\nu \approx Y(\nu)\Theta$ (regressor form) and lump the unknowns. Extend the CLF:

$$V_c = V_1 + V_2 + \frac{1}{2}\tilde{\Theta}^\top \Gamma_\theta^{-1} \tilde{\Theta} + \frac{1}{2}\tilde{\tau}_d^\top \Gamma_d^{-1} \tilde{\tau}_d$$

Choosing the **adaptive laws**:

$$\dot{\hat{\Theta}} = \Gamma_\theta\, Y(\nu)^\top z_2, \qquad \dot{\hat{\tau}}_d = \Gamma_d\, z_2$$

cancels all parameter-error terms from $\dot{V}_c$.

### Full Control Law

$$
\boxed{\tau = -J_e(q_e) z_1 - K_2 z_2 + M\dot{\alpha} + C(\nu)\nu - Y(\nu)\hat{\Theta} - \hat{\tau}_d}
$$

which gives $\dot{V}_c = -K_1 z_1^\top z_1 - K_2 z_2^\top z_2 < 0 \quad \forall z \neq 0 \quad \blacksquare$

### Computing $\dot{\alpha}$

The tricky part is $\dot{\alpha} = \dot{\alpha}_1 + \dot{\alpha}_2$ where $\alpha = -J_e(q_e)^{-1} K_1 z_1$:

$$\dot{\alpha}_1 = -\dot{J}_e(q_e)^{-1} K_1 z_1, \qquad \dot{\alpha}_2 = -J_e(q_e)^{-1} K_1 \dot{z}_1$$

$\dot{\alpha}_2$ is straightforward since $\dot{z}_1 = J_e(\eta)\nu$. For $\dot{\alpha}_1$, we need $\dot{J}_e$, which involves:

$$\dot{R}(q^n) = R(q^n) S(\omega_{nb}^b), \qquad \dot{T}_e = \dot{\eta}_e I_3 + S(\dot{\varepsilon}_e)$$

where $\dot{\eta}_e$ and $\dot{\varepsilon}_e$ come from differentiating $\delta q = q_d^* \otimes q$:

$$\dot{q}_e = \begin{bmatrix} \dot{\eta}_e \\ \dot{\varepsilon}_e \end{bmatrix} = \frac{1}{2} q_e \otimes \begin{bmatrix} 0 \\ \omega \end{bmatrix} = \frac{1}{2}\begin{bmatrix} -\varepsilon_e^\top \omega \\ (\eta_e I_3 + S(\varepsilon_e))\omega \end{bmatrix}$$

Substituting back yields $\dot{T}_e = \frac{1}{2}\begin{bmatrix} \varepsilon_e^\top \omega \\ T_e(q_e)\omega \end{bmatrix}$ and the full $\dot{\alpha}_1 = J_e^{-1} \dot{J}_e J_e^{-1} K_1 z_1$, where:

$$\dot{J}_e = \begin{bmatrix} R S(\omega) & 0 \\ 0 & \frac{1}{2}(S(T_e \omega) - \varepsilon_e^\top \omega I_3) \end{bmatrix}$$

The $T_e$ matrix was deliberately designed to make this expression tractable.

---

## No Gimbal Lock at 90° Pitch

Euler-angle-based controllers lose a degree of freedom at ±90° pitch (gimbal lock). The quaternion error-state formulation has no such singularity — $J_e$ remains full rank for all $|\delta\theta| < 180°$.

Below: the AUV commanded to hold 90° pitch in simulation, and the corresponding tracking data.

![AUV at 90° pitch experiencing no gimbal lock](gimbal_lock.jpg)

---

## Links

- [Source code (ROS2)](https://github.com/vortexntnu/vortex-auv/tree/main/control/dp_adapt_backs_controller_quat)
- [Vortex NTNU](https://www.vortexntnu.no/)

## Sources

