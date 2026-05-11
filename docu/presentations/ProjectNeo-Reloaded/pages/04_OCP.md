---
title: OCP - Variable
class: text-right
layout: image-right
image: /jones.webp
---

<Footnotes x='l'>
    <Footnote>filmfreedonia.com</Footnote>
</Footnotes>

<div class="h-20"/>

# OCP Formulation 📈

<div class="h-2"/>

$$
\begin{align}
x(t) &=
\begin{bmatrix}
q(t) \\ \dot q(t) \\
\end{bmatrix}
\\
\nonumber \\
u(t) &= \ddot q(t) \\
\nonumber \\
f_{expl}(x(t), u(t), t) & =
\begin{bmatrix}
\dot q(t) \\ \ddot q(t) \\
\end{bmatrix} \\
&=\, \dot x(t) \nonumber
\end{align}
$$

---
title: OCP - Core
class: text-center
---

<div class="h-10"/>
<h1 class="text-center">OCP Formulation 📈</h1>
<div class="h-2"/>

<v-switch>


  <template #0>

$$
\begin{align}
\min_{x(.),\ u(.)} & \int_0^{T_f} \omega_s l_s(x(t),u(t))+\omega_h l_h(x(t),t)+\omega_r l_r(u(t)) dt \tag{4} \\
\nonumber \\
\text{s.t.} \qquad \qquad
\begin{bmatrix}
q_0 \\ 0 \\
\end{bmatrix}
&= x(0) \tag{C0} \\
\begin{bmatrix}
\underline q(t) \\ -v_{max}\\
\end{bmatrix} &
\leq x(t) \leq
\begin{bmatrix}
\overline q(t) \\ v_{max}\\
\end{bmatrix}
&\forall t \in [0, T_f] \tag{C1}
\\
-a_{max} & \leq u(t) \leq a_{max}
& \forall t \in [0, T_f] \tag{C2} \\
p_{stab} (t) & \in \Rho &\forall t \in [0, T_f] \tag{C3} \\
\end{align}
$$

  </template>
  <template #1>

$$
\begin{align}
\min_{x(.),\ u(.)} & \int_0^{T_f} \omega_s l_s(\textcolor{#b000ff}{x(t)},u(t))+\omega_h l_h(\textcolor{#b000ff}{x(t)},t)+\omega_r l_r(u(t)) dt \tag{4} \\
\nonumber \\
\text{s.t.} \qquad \qquad
\begin{bmatrix}
q_0 \\ 0 \\
\end{bmatrix}
&= \textcolor{#b000ff}{x(0)} \tag{C0} \\
\begin{bmatrix}
\underline q(t) \\ -v_{max}\\
\end{bmatrix} &
\leq \textcolor{#b000ff}{x(t)} \leq
\begin{bmatrix}
\overline q(t) \\ v_{max}\\
\end{bmatrix}
&\forall t \in [0, T_f] \tag{C1}
\\
-a_{max} & \leq u(t) \leq a_{max}
& \forall t \in [0, T_f] \tag{C2} \\
p_{stab} (t) & \in \Rho &\forall t \in [0, T_f] \tag{C3} \\
\end{align}
$$

  </template>
  <template #2>

$$
\begin{align}
\min_{x(.),\ u(.)} & \int_0^{T_f} \omega_s l_s(\textcolor{#b000ff}{x(t)},\textcolor{#0092ff}{u(t)})+\omega_h l_h(\textcolor{#b000ff}{x(t)},t)+\omega_r l_r(u(t)) dt \tag{4} \\
\nonumber \\
\text{s.t.} \qquad \qquad
\begin{bmatrix}
q_0 \\ 0 \\
\end{bmatrix}
&= \textcolor{#b000ff}{x(0)} \tag{C0} \\
\begin{bmatrix}
\underline q(t) \\ -v_{max}\\
\end{bmatrix} &
\leq \textcolor{#b000ff}{x(t)} \leq
\begin{bmatrix}
\overline q(t) \\ v_{max}\\
\end{bmatrix}
&\forall t \in [0, T_f] \tag{C1}
\\
-a_{max} & \leq \textcolor{#0092ff}{u(t)} \leq a_{max}
& \forall t \in [0, T_f] \tag{C2} \\
p_{stab} (t) & \in \Rho &\forall t \in [0, T_f] \tag{C3} \\
\end{align}
$$

  </template>
  <template #3>

$$
\begin{align}
\min_{x(.),\ u(.)} & \int_0^{T_f} \omega_s l_s(\textcolor{#b000ff}{x(t)},\textcolor{#0092ff}{u(t)})+\omega_h l_h(\textcolor{#b000ff}{x(t)},t)+\omega_r l_r(u(t)) dt \tag{4} \\
\nonumber \\
\text{s.t.} \qquad \qquad
\begin{bmatrix}
q_0 \\ 0 \\
\end{bmatrix}
&= \textcolor{#b000ff}{x(0)} \tag{C0} \\
\begin{bmatrix}
\underline q(t) \\ -v_{max}\\
\end{bmatrix} &
\leq \textcolor{#b000ff}{x(t)} \leq
\begin{bmatrix}
\overline q(t) \\ v_{max}\\
\end{bmatrix}
&\forall t \in [0, T_f] \tag{C1}
\\
-a_{max} & \leq \textcolor{#0092ff}{u(t)} \leq a_{max}
& \forall t \in [0, T_f] \tag{C2} \\
\textcolor{#ff6a00}{p_{stab} (t)} & \in \Rho &\forall t \in [0, T_f] \tag{C3} \\
\end{align}
$$

  </template>
  <template #4>

$$
\begin{align}
\min_{x(.),\ u(.)} & \int_0^{T_f} \textcolor{#ff6a00}{\omega_s l_s}(x(t),u(t))+\textcolor{#b9d080}{\omega_h l_h}(x(t),t)+\textcolor{#0092ff}{\omega_r l_r}(u(t)) dt \tag{4} \\
\nonumber \\
\text{s.t.} \qquad \qquad
\begin{bmatrix}
q_0 \\ 0 \\
\end{bmatrix}
&= x(0) \tag{C0} \\
\begin{bmatrix}
\underline q(t) \\ -v_{max}\\
\end{bmatrix} &
\leq x(t) \leq
\begin{bmatrix}
\overline q(t) \\ v_{max}\\
\end{bmatrix}
&\forall t \in [0, T_f] \tag{C1}
\\
-a_{max} & \leq u(t) \leq a_{max}
& \forall t \in [0, T_f] \tag{C2} \\
p_{stab} (t) & \in \Rho &\forall t \in [0, T_f] \tag{C3} \\
\end{align}
$$

  </template>
</v-switch>

---
title: OCP - Cost Function
class: text-center
---

<div class="h-30"/>
<h1 class="text-center">Lagrange Cost Functions 📈</h1>
<div class="h-2"/>

<v-switch>
  <template #0>

$$
\begin{align}
l_h(x(t),t) &: \text{🎯 Head Motion Tracking} \tag{L0} \\
\nonumber \\
l_s(x(t), u(t)) &: \text{⚖️ Stability Metric} \tag{L1} \\
\nonumber \\
l_r(u(t)) &: \text{🎚️ Regularisation Term} \tag{L2}
\end{align}
$$

  </template>
  <template #1>

$$
\begin{align}
l_h(x(t),t) &: \text{🎯 Head Motion Tracking} \tag{L0} \\
\nonumber \\
l_s(x(t), u(t)) &: \text{⚖️ Stability Metric} \tag{L1} \\
\nonumber \\
l_r(u(t)) &= u(t)^2 \tag{L2}
\end{align}
$$

  </template>

</v-switch>

---
title: Head Motion Tracking
class: text-center
layout: image-right
image: /head_motion.png
---

<div class="h-40"/>

# Head Motion Tracking 🎯

<div class="h-2"/>

<v-switch>
  <template #0>

$$
\begin{align}
l_h(x(t),t) &= (p^z_{head}(x(0))-\Delta h\ t \nonumber \\
&-p^z_{head}(x(t))^2 \tag{L0}  \\
\end{align}
$$

  </template>
  <template #1>

$$
\begin{align}
l_h(x(t),t) &= (\textcolor{#0093ff}{p^z_{head}(x(0))}-\textcolor{#0093ff}{\Delta h\ t} \nonumber \\
&\textcolor{#ff6a00}{-p^z_{head}(x(t)})^2 \tag{L0}  \\
\end{align}
$$

  </template>
</v-switch>

<span class="absolute right-14em top-14em">$t$</span>
<span class="absolute right-20em top-12em">$\Delta h$</span>

<span class="absolute right-0.5em top-16em">$p^z_{head}(x(t))$</span>
<span class="absolute right-16.5em top-7.5em">$p^z_{head}(x(0))$</span>

---
title: OCP - Cost Function
class: text-center
---

<div class="h-30"/>
<h1 class="text-center">Lagrange Cost Functions 📈</h1>
<div class="h-2"/>

$$
\begin{align}
l_h(x(t),t) &= (p^z_{head}(x(t))-\Delta h\ t -p^z_{head}(x(0)))^2 \tag{L0}  \\
\nonumber \\
l_s(x(t), u(t)) &: \text{⚖️ Stability Metric} \tag{L1} \\
\nonumber \\
l_r(u(t)) &= u(t)^2 \tag{L2}
\end{align}
$$

<div v-click="['1','2']" class="border-2 border-solid border-lime500 absolute left-14em bottom-8.4em" style="width:30em; height:2.5em;"></div>
<div v-click="['1','2']" class="border-2 border-solid border-lime500 absolute left-14em bottom-15.6em" style="width:30em; height:2.5em;"></div>
<div v-click="2" class="border-2 border-solid border-purple500 absolute left-14em bottom-12.1em" style="width:20em; height:2.5em;"></div>
