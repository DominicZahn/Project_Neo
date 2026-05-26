---
title: H1-2 Example Showcase
layout: image-right
image: /recordings/h1-2-duck-vertical.GIF
backgroundSize: contain
class: text-right
---
<div class="h-50"/>

# Ducking H1-2

---
title: Simple Example Showcase
layout: image-right
image: /recordings/arm-vertical.GIF
backgroundSize: contain
class: text-center
---
<div class="h-50"/>

# Simple Example
#### 3-Segment Arm

---
title: OCP Formulation
layout: image-right
image: /recordings/arm-vertical.png
class: text-right
---

<div class="h-20"/>

# OCP Formulation

<div class="h-2"/>

$$
\begin{align}
x(t) &=
\begin{bmatrix}
q(t) \\ \dot q(t) \\
\end{bmatrix}
\\
\nonumber \\
u(t) &= \tau(t) \\
\nonumber \\
f_{expl}(x(t), u(t), t) & =
\begin{bmatrix}
\dot q(t) \\ \ddot q(t) \\
\end{bmatrix} \\
&=\, \dot x(t) \nonumber
\end{align}
$$

<!-- highlighter -->
<div v-click="['1','2']" class="border-2 border-solid border-purple500 absolute left-12em top-10em text-purple500" style="width:15em; height:5.7em;">States &nbsp</div>
<div v-click="['2','3']" class="border-2 border-solid border-purple500 absolute left-12em top-15em text-purple500" style="width:15em; height:4.2em;">Controls &nbsp</div>
<div v-click="3" class="border-2 border-solid border-purple500 absolute left-6em top-19em text-purple500" style="width:21em; height:6.8em;">Dynamics &nbsp</div>

---
title: OCP Formulation
layout: image-right
image: /recordings/arm-vertical.png
class: text-right
---

<div class="h-20"/>

# OCP Formulation

<div class="h-2"/>

$$
\begin{align}
\min_{x(.),\ u(.)} & \int_0^{T_f} \| p(t) - p_{\text{target}} \|^2_{I_3} \ dt \qquad \\
\nonumber \\
\text{s.t.} \qquad \qquad \quad
&\quad \ x(0) = 0_{N_q} \\
\begin{bmatrix}
-v_{\text{max}} \\
\end{bmatrix}_{N_q} &
\leq
\dot q(t) \leq
\begin{bmatrix}
 v_{\text{max}}\\
\end{bmatrix}_{N_q} \\
\left[-\tau_{\text{max}}\right]_{N_q} & \leq u(t) \leq \left[\tau_{\text{max}}\right]_{N_q} \\
\left[-v_\text{static}\right]_{N_q} & \leq \dot q(T_f) \leq \left[v_\text{static}\right]_{N_q} \\
\end{align}
$$

<!-- highlighter -->
<div v-click="['1','2']" class="border-2 border-solid border-purple500 absolute left-5.1em top-10em text-purple-500" style="width:20.2em; height:5.7em;">Objective Function &nbsp</div>
<div v-click="2" class="border-2 border-solid border-purple500 absolute left-5.1em top-15.3em text-purple500" style="width:20.2em; height:10em;">Constraints &nbsp</div>


---
title: Pipeline (RBD)
class: text-center
---

<div class="h-20"/>

# Pipeline

<div class="h-10"/>
<v-switch>
  <template #0>

```mermaid
flowchart LR
  classDef body fill:white
  classDef none stroke:white,fill:white

    p["<wbr><img src="/TechStack/pinocchio_logo.png"/>Rigid Body Dynamics"]
    c["<wbr><img src="/TechStack/casadi.png"/>Computer Algebra"]
    a["<wbr><img src="/TechStack/acados_logo.png"/>Optimal Control"]

     p:::body --- c:::none -->a:::body
```
  </template>
  <template #1>

```mermaid
flowchart LR
  classDef body fill:white
  classDef highlight fill:white,stroke:#0093ff,stroke-width:2px
  classDef none stroke:white,fill:white

    p["<wbr><img src="/TechStack/pinocchio_logo.png"/>Rigid Body Dynamics"]
    c["<wbr><img src="/TechStack/casadi.png"/>Computer Algebra"]
    a["<wbr><img src="/TechStack/acados_logo.png"/>Optimal Control"]

     p:::highlight --- c:::none -->a:::body
```
  </template>
</v-switch>

