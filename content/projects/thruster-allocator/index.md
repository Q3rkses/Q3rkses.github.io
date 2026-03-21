---
title: "Thrust Allocator — Vortex NTNU"
date: 2026-03-21
draft: false
summary: "Constrained and pseudoinverse thrust allocation for Vortex NTNU's AUV, based on Fossen's formulations."
tags: ["control-systems", "robotics", "ROS2", "optimization", "QP"]
---

{{< katex >}}

## Overview

The **Thrust Allocator** distributes generalized control forces $\tau \in \mathbb{R}^n$ to the AUV's actuators as control inputs $u \in \mathbb{R}^r$. For linear systems this reduces to $\tau = Bu$, where $B$ is the input matrix. The individual control inputs $u_i$ are passed downstream to the thruster interface.

This module is part of [Vortex NTNU's](https://www.vortexntnu.no/) autonomous underwater vehicle stack.

---

## Solvers

### Pseudoinverse Allocator

Follows the unconstrained weighted least-squares formulation from Fossen (2021, Eq. 11.27):

$$
J = \min_{f_e} \; f_e^\top W_f f_e
\qquad \text{s.t.} \qquad
\tau - T f = 0
$$

Solving yields the **generalized pseudoinverse** (Fossen Eq. 11.35):

$$
T_w^+ = W_f^{-1} T_e^\top \left( T_e W_f^{-1} T_e^\top \right)^{-1}
$$

Since the Orca drone uses 8 identical thrusters, there is no practical reason to weight actuators differently. With $W_f = I$, the solution simplifies to the **right Moore–Penrose pseudoinverse** (Fossen Eq. 11.36)
Our new drone, however, will utilize 8 thrusters and they will have different maximum and minimum thrust in the forward and backward direction. To keep optimality a more advanced method could be utilized

$$
T^+ = T_e^\top (T_e T_e^\top)^{-1}
$$

### Constrained QP Allocator

Formulated as a quadratic program following Fossen (2021, Eq. 11.38). The original formulation includes a load-balancing parameter $\bar{f}$, but for our use case different maneuvers require some thrusters to work hard while others rest — so load balancing does more harm than good.

The implemented formulation drops the load-balancing term:

$$
J = \min_{f_e,\, s} \; \left( f_e^\top W_f f_e + s^\top Q s \right)
$$

$$
\text{s.t.} \quad T_e f_e = \tau + s, \qquad f_{\min} \le f_e \le f_{\max}
$$

This retains thrust limits and soft constraint handling via the slack vector $s$ while avoiding unnecessary force redistribution.

---

## Notation

All formulations follow the notation of Fossen (2021), Ch. 11:

- $\tau \in \mathbb{R}^n$ — desired generalized force
- $T_e$ — extended thruster configuration matrix
- $f_e$ — extended force vector
- $W_f \succeq 0$ — weighting matrix on the extended force vector
- $Q \succeq 0$ — weighting matrix on the slack vector $s$
- $f_{\min}, f_{\max}$ — lower and upper bounds on $f_e$

---

## Acknowledgements

The thrust allocation formulations are based on:

> **Thor I. Fossen**, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd Edition, Wiley, 2021 — Chapter 11.

---

## Links

- [Vortex NTNU](https://www.vortexntnu.no/)
- [Source code](https://github.com/vortexntnu/vortex-auv/tree/main/motion/thrust_allocator_auv)
- [Thruster Interface](https://github.com/vortexntnu/vortex-auv/tree/main/motion/thruster_interface_auv)
