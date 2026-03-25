---
level: 2
class: text-left
layout: two-cols
---

# <font color="#0093ff">Previous</font> Criteria 📐

<div class="h-30"/>

<h2 v-click="1" v-mark="{ at: 3, color: '#ff6a00', type: 'box'}" class="h-20"> 📏 no normalization</h2>
<h2 v-click="1" v-mark="{ at: 3, color: '#ff6a00', type: 'box'}" class="h-20"> 🚧 no limitation to PoS</h2>
<h2 v-click="2" class="h-20"> 🌳 projected CoM <i>(static)</i></h2>

::right::

<div class="text-right h-23">
🔴: low values | 🔵: high values
</div>

<img class="mx-auto block" width="95%" src="/centroidDst.png"/>
<h2 class="text-center" v-mark="{ at: 3, color: '#ff6a00', type: 'box'}"> 🎯 Centroid Distance</h2>


---
title: Stability Metric
class: text-center
layout: figure
figureUrl: /parabelStability.gif
---

# Stability Metric ⚖️

---
title: OCP - Full Cost Function
class: text-center
---

<div class="h-30"/>
<h1 class="text-center">Lagrange Cost Functions 📈</h1>
<div class="h-2"/>

<v-switch>
  <template #0>

$$
\begin{align}
l_h(x(t),t) &= (p^z_{head}(x(t))-\Delta h\ t -p^z_{head}(x(0)))^2 \tag{L0} \\
\nonumber \\
l_s(x(t), u(t)) &= \frac{(p_{stab}(x(t),u(t))-\Rho_c)^2}{(\underline\Rho-\Rho_c)^2} \tag{L1} \\
\nonumber \\
l_r(u(t)) &= u(t)^2 \tag{L2}
\end{align}
$$

  </template>
  <template #2>

$$
\begin{align}
\textcolor{#83cb1c}{l_h(x(t),t)} &= (p^z_{head}(x(t))-\Delta h\ t -p^z_{head}(x(0)))^2 \tag{L0} \\
\nonumber \\
\textcolor{#83cb1c}{l_s(x(t), u(t))} &= \frac{(p_{stab}(x(t),u(t))-\Rho_c)^2}{(\underline\Rho-\Rho_c)^2} \tag{L1} \\
\nonumber \\
\textcolor{#83cb1c}{l_r(u(t))} &= u(t)^2 \tag{L2}
\end{align}
$$

  </template>
</v-switch>
<div v-click="[1,2]" class="border-2 border-solid border-lime500 absolute left-14em bottom-10.5em" style="width:30em; height:4.1em;"></div>

---
title: Static Stability Results
class: text-center
layout: two-cols-header
---

# 🌳 Projected CoM

::left::

<img src="/recordings/bob15_cost.png" width="100%"/>
<img src="/recordings/bob15_acados.png" width="100%"/>

::right::
<div class="h-5"/>
<img src="/recordings/bob15.GIF" width="55%"/>


---
title: Dynamcis Stability Results
class: text-center
layout: two-cols-header
---

# 🤸 Zero Moment Point (ZMP)

::left::

<img src="/recordings/jackson15_cost.png" width="100%"/>
<img src="/recordings/jackson15_acados.png" width="100%"/>

::right::
<div class="h-5"/>
<img src="/recordings/jackson15.GIF" width="55%"/>

---
title: Result Comparison
class: text-center
layout: two-cols-header
---


::left::

# 🌳 Projected CoM

<img src="/recordings/bob15_cost.png" width="100%"/>
<img src="/recordings/bob15_acados.png" width="100%"/>

::right::

# 🤸 Zero Moment Point
<img src="/recordings/jackson15_cost.png" width="100%"/>
<img src="/recordings/jackson15_acados.png" width="100%"/>

---
title: Dynamic Equations
class: text-center
---
<div class="grid items-center grid-cols-2 gap-x-50 gap-y-7">
  <h1 v-mark="{ at: [1], color: '#ff6a00', type: 'box'}" class="mx-auto block"> 🤸 Full ZMP</h1>
  <h1 class="mx-auto block"> 🪀 Approximated ZMP</h1>

  <img class="mx-auto block" src="/inertia_cut.png" width="40%"/>
  <img class="mx-auto block" src="/coms_cut.png" width="40%"/>
  <div v-mark="{ at: [2], color: '#b000ff', type: 'box'}">

$$
\begin{align}
p_x = \frac{m_{tot}\,g\,x + p_z\,\dot M_T^x - \dot M_A^y}{m_{tot}\,g + \dot M_T^z} \nonumber \\
\end{align}
$$
  </div>
  <div v-mark="{ at: 3, color: '#0092ff', type: 'box'}" class="mx-auto block">

$$
\begin{align}
p_x = \frac{\sum^N_{i=0}m_i((\ddot z_i+g)x_i-(z_i-p_z)\ddot x_i)}{\sum^N_{i=0}m_i(\ddot z_i+g)} \nonumber \\
\end{align}
$$
  </div>
</div>

<div class="absolute left-17.5em top-8.75em">

$$
\begin{align}
m_{tot}&: \text{Total Mass} & \left[kg\right] \nonumber \\
m_i&: \text{Segment Mass} & \left[kg\right] \nonumber \\
x, y, z&: \text{CoM} & \left[ m \right] \nonumber \\
g&: \text{Grav. Acc.} & \left[ m / s^2\right] \nonumber \\
M_A&: \text{Angular Moment} & \left[ Nms\right] \nonumber \\
M_T&: \text{Translational Moment} & \left[ Ns\right] \nonumber \\
\end{align}
$$

</div>


<Footnotes x='l'>
<Footnote>Kajita - Introduction to Humanoid Robotics Ch.3</Footnote>
</Footnotes>

---
title: Static Stability Results
class: text-center
layout: two-cols-header
---

# 🪀 Approximated ZMP

::left::
<img src="/recordings/jackson15approx_cost.png" width="100%"/>
<img src="/recordings/jackson15approx_acados.png" width="100%"/>

::right::
<div class="h-5"/>
<img src="/recordings/jackson15approx.GIF" width="55%"/>

---
title: Dynamic Result Comparison
class: text-center
layout: two-cols-header
---

::left::
# 🤸 Full ZMP
<img src="/recordings/jackson15_cost.png" width="100%"/>
<img src="/recordings/jackson15_acados.png" width="100%"/>

::right::
# 🪀 Approximated ZMP
<v-switch>
  <template #0>
    <img src="/recordings/jackson15approx_cost.png" width="100%"/>
  </template>
  <template #1>
    <img src="/recordings/jackson15approx_cost_crop.png" width="100%"/>
  </template>
</v-switch>
<img src="/recordings/jackson15approx_acados.png" width="100%"/>


