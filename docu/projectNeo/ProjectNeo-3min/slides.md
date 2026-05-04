---
theme: ./theme
themeConfig:
  paginationX: l
  paginationY: b
  paginationPagesDisabled: 1 2 6
info: |
  ## 3min Presetionation for Project Neo

drawings:
  persist: false
mdc: true
colorSchema: light
duration: 3min

layout: center
class: text-center
title: Project Neo
hideInToc: true
transition: slide-left
---
<!--
# What have all these videos in common?
-->

<style>
.image-container {
    text-align: right;
}

.footnote-number {
    font-size: 0.4em;
    display: block;
    margin-top: 0.2rem;
}
</style>

<div class="grid grid-cols-4 gap-x-12 gap-y-12">
  <div class="image-container">
    <img src="/push/push_atlas.webp"/>
    <span class="footnote-number">1</span>
  </div>
  <div class="image-container">
    <img src="/push/push_op3.webp"/>
    <span class="footnote-number">2</span>
  </div>
  <div class="image-container">
    <img src="/push/push_tuDelft.webp"/>
    <span class="footnote-number">3</span>
  </div>
  <div class="image-container">
    <img src="/push/push_oldSpot2.webp"/>
    <span class="footnote-number">4</span>
  </div>
  <div class="image-container">
    <img src="/push/push_atrias.webp"/>
    <span class="footnote-number">5</span>
  </div>
  <div class="image-container">
    <img src="/push/push_g1.webp"/>
    <span class="footnote-number">6</span>
  </div>
  <div class="image-container">
    <img src="/push/push_reddit.webp"/>
    <span class="footnote-number">7</span>
  </div>
  <div class="image-container">
    <img src="/push/push_atlas2.webp"/>
    <span class="footnote-number">8</span>
  </div>
  <div class="image-container">
    <img src="/push/push_bosstownDynamics.webp"/>
    <span class="footnote-number">9</span>
  </div>
  <div class="image-container">
    <img src="/push/push_nadia.webp"/>
    <span class="footnote-number">10</span>
  </div>
  <div class="image-container">
    <img src="/push/push_agilityRobotics.webp"/>
    <span class="footnote-number">11</span>
  </div>
  <div class="image-container">
    <img src="/push/push_oldSpot.webp"/>
    <span class="footnote-number">12</span>
  </div>
</div>

<Footnotes x='l' seperator>
    <Footnote :number=1>CNN</Footnote>
    <Footnote :number=2>OP3 Soccer</Footnote>
    <Footnote :number=3>TU Delft</Footnote>
    <Footnote :number=4>CNN</Footnote>
    <Footnote :number=5>OSU</Footnote>
    <Footnote :number=6>Unitree</Footnote>
    <Footnote :number=7>OCP3 Soccer</Footnote>
    <Footnote :number=8>CNN</Footnote>
    <Footnote :number=9>Corridor Digital</Footnote>
    <Footnote :number=10>IHMC Robotics</Footnote>
    <Footnote :number=11>Agility Robotics</Footnote>
    <Footnote :number=12>CNN</Footnote>
</Footnotes>

<!-- Sources:
https://www.youtube.com/watch?v=0VgxAnZKM14 CNN
https://www.youtube.com/watch?v=7b53L10RaIE (OSU) Atrias
https://www.youtube.com/watch?v=LfcZ_EdTtGw G1 (unitree)
https://www.reddit.com/r/oddlyterrifying/comments/1j6h9q9/how_quickly_this_robot_gets_up_after_being_pushed/ reddit
https://www.youtube.com/watch?v=KSvLcr5HtNc OP3 Soccer
https://www.youtube.com/watch?v=y3RIHnK0_NE&t=101s Bosstown Dynamics (Corridor Digital)
https://www.youtube.com/watch?v=aM-qb1yd5mU nadia (IHMC Robotics)
https://www.youtube.com/watch?v=2amzGvk97GE Agility Robotics
https://www.youtube.com/watch?v=w97H0eEKYvY TU Delft
-->

---
layout: cover
class: text-right text-white
title: Project Neo
hideInToc: true
transition: slide-left
coverAuthor: Dominic Zahn
coverDate: 5/5/2026
coverBackgroundUrl: /bullet_dodge_neo.webp
---

# <span style="color: lightgray"><b>Project Neo</b></span>

<!--
<span style="color: white">No bullet-time for **H**e**1**nz</span>
-->

---
layout: image-right
image: /dojo_render.jpg
class: text-center
---

<div class="h-5"/>
<div class="grid items-center grid-cols-3 gap-x-15 gap-y-10" style="display: flex; justify-content: center">
  <img width="100em" src="/TechStack/gz.svg"/>
  <img width="100em" src="/TechStack/Docker.svg"/>
  <img width="100em" src="/TechStack/ROS.svg"/>
</div>
<div class="h-5"/>

![Repo Card](https://github-readme-stats.vercel.app/api/pin/?username=DominicZahn&repo=Neo-Construct)
<a style="font-size: 12px; color: gray " href="https://gitlab.kit.edu/kit/iar-hcr/frameworks/neoConstruct">https://gitlab.kit.edu/kit/iar-hcr/frameworks/neoConstruct</a>

<div class="h-10"/>
<div class="grid items-center grid-cols-2 gap-x-10 gap-y-10" style="display: flex; justify-content: center">
  <img width="200em" src="/TechStack/pinocchio_logo.png"/>
  <img width="200em" src="/TechStack/acados_logo.png"/>
</div>

<!-- Additional Packages -->
<div v-click.scale="1" class="absolute left-5em bottom-8em">
  <img width="100em" src="/TechStack/rbdl.svg"/>
</div>
<div v-click.scale="1" class="absolute left-17em bottom-7.2em">
  <img width="90em" src="/TechStack/rviz.png"/>
</div>

<div v-click.scale="1" class="absolute left-3em bottom-0em">
  <img width="80em" src="/TechStack/boost.svg"/>
</div>
<div v-click.scale="1" class="absolute left-10.5em bottom-0em">
  <img width="80em" src="/TechStack/eigen.png"/>
</div>
<div v-click.scale="1" class="absolute left-18em bottom-1em">
  <img width="130em" src="/TechStack/nlopt.png"/>
</div>

<Footnotes x='r'>
    <Footnote style="color: gray">learnvray.com</Footnote>
</Footnotes>

---
layout: image-right
image: /recordings/5.GIF
class: text-center
---

<div class="h-50"/>
<h1><code>H</code>e<code>1</code>nz<br>the Chosen One</h1>

---
layout: cover
coverDate: .
coverBackgroundUrl: /recordings/fails.GIF
---

# <span style="color: lightgray"><b>Questions?</b></span>

<!-- APPENDIX -->

---
layout: center
---

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
\end{bmatrix}
\end{align}
$$

$$
\begin{align}
\min_{x(.),\ u(.)} & \int_0^{T_f} \omega_s l_s(x(t),u(t))+\omega_h l_h(x(t),t)+\omega_r l_r(u(t)) dt \qquad\qquad \tag{Obj.} \\
\nonumber \\
\text{s.t.} \qquad \qquad
\begin{bmatrix}
q_0 \\ 0 \\
\end{bmatrix}
&= x(0) \tag{Cons. 0} \\
\begin{bmatrix}
\underline q(t) \\ -v_{max}\\
\end{bmatrix} &
\leq x(t) \leq
\begin{bmatrix}
\overline q(t) \\ v_{max}\\
\end{bmatrix}
\qquad\qquad\qquad\ \,\forall t \in [0, T_f] \tag{Cons. 1}
\\
-a_{max} & \leq u(t) \leq a_{max}
\qquad\qquad\qquad\quad\ \,\, \forall t \in [0, T_f] \tag{Cons. 2} \\
p_{stab} (t) & \in \Rho \qquad\qquad\qquad\qquad\qquad\qquad \forall t \in [0, T_f] \tag{Cons. 3} \\
\end{align}
$$

<h1 class="text-center absolute left-2em top-2em">OCP<br>Formulation</h1>
