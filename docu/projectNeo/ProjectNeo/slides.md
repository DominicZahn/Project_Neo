---
theme: academic
themeConfig:
    paginationX: r
    paginationY: b
    paginationPagesDisabled: 1 2 3

info: |
    ## Presentation for ProjectNeo
    Showcases, which systems were build and what scientific advancements were made.
drawings:
  persist: false
mdc: true
colorSchema: light


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
    <img src="/push_atlas.webp"/>
    <span class="footnote-number">1</span>
  </div>
  <div class="image-container">
    <img src="/push_op3.webp"/>
    <span class="footnote-number">2</span>
  </div>
  <div class="image-container">
    <img src="/push_tuDelft.webp"/>
    <span class="footnote-number">3</span>
  </div>
  <div class="image-container">
    <img src="/push_oldSpot2.webp"/>
    <span class="footnote-number">4</span>
  </div>
  <div class="image-container">
    <img src="/push_atrias.webp"/>
    <span class="footnote-number">5</span>
  </div>
  <div class="image-container">
    <img src="/push_g1.webp"/>
    <span class="footnote-number">6</span>
  </div>
  <div class="image-container">
    <img src="/push_reddit.webp"/>
    <span class="footnote-number">7</span>
  </div>
  <div class="image-container">
    <img src="/push_atlas2.webp"/>
    <span class="footnote-number">8</span>
  </div>
  <div class="image-container">
    <img src="/push_bosstownDynamics.webp"/>
    <span class="footnote-number">9</span>
  </div>
  <div class="image-container">
    <img src="/push_nadia.webp"/>
    <span class="footnote-number">10</span>
  </div>
  <div class="image-container">
    <img src="/push_agilityRobotics.webp"/>
    <span class="footnote-number">11</span>
  </div>
  <div class="image-container">
    <img src="/push_oldSpot.webp"/>
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
title: Bullet Dodge
hideInToc: true
layout: image
image: /bullet_dodge_neo.webp
---
<!--
TODO: make picture darker and put text "Dodging" in the middle
-->
---
title: Cover Page
hideInToc: true
layout: cover

coverAuthor: Dominic Zahn
coverAuthorUrl: https://github.com/DominicZahn
coverDate: 8/29/2025
#coverBackgroundUrl: /background_3.jpg
coverBackgroundUrl: /bullet_dodge_neo.webp

class: text-right text-white
---

# <span style="color: white">PROJECT NEO</span>
<span style="color: white">Getting **H**e**1**nz to dodge like the chosen one.</span>

<!--
Put your notes here!
-->
---
title: Dodging is Underrepresented
layout: two-cols-header
hideInToc: true
class: text-center
---
# Dodging is Underrepresented
::left::
<img align="right" src="/google-scholar-icon.png" width="350"/>

::right::
```mermaid
%%{
  init: {
    'theme': 'base',
    'themeVariables': {
      'primaryColor': '#2596be',
      'primaryTextColor': '#000',
      'primaryBorderColor': '#7C0000',
      'secondaryColor': '#b04d1e',
      'tertiaryColor': '#bea925'
    }
  }
}%%
pie showData
    title Humanoid ...
    "dodge OR dodging" : 3840
    "push recovery" : 18200
```
<!--
data from 27.8. 17:00
-->

---
title: Why Dodging Bullets is Interesting 💡
level: 1
layout: image-right
image: /shooting_at_robot.png
class: text-right
hideInToc: true
---
# Why Dodging Bullets is Intresting

<v-clicks>

**No Killer-Robots** ❌

**Effective Evasion Manauever** ✅

**Avoids Damage** ✅

**Investigating Stability Criterias** ✅

**Applyed Mathematical Optimization** ✅

**Exploring Platform Limitations** ✅
<!--
> **More Fun than Getting Hit** 🤖
-->

> *"I don`t like to get hit, who likes it?"*
>
> -- Wladimir Klitschko

</v-clicks>

---
layout: image-right
class: text-right
image: /jones.webp
hideInToc: true
---
# Outline

<div class="grid grid-cols-[60%_35%_1fr] gap-y-3">
    <div>
        Unitree H1 ↔
    </div>
    <div text-align="left">
        <b>Powerfull Platform</b> 
    </div>
    <div>🦾</div>
    <hr style="grid-column: 1 / -1; border: none; height: 1px; background-color: #333;">
    <div>
        Docker ↔
    </div>
    <div text-align="left">
        <b>Portable Environment</b> 
    </div>
    <div>📦</div>
    <div>
        ROS 2 ↔
    </div>
    <div text-align="left">
        <i>Jazzy</i> 
    </div>
    <div>🧪</div>
    <hr style="grid-column: 1 / -1; border: none; height: 1px; background-color: #333;">
    <div>
    </div>
    <div text-align="left">
        <b>Toolbox</b> 
    </div>
    <div>🧰</div>
    <div>
        Optimization  ↔
    </div>
    <div text-align="left">
        <i>NLOPT</i>
    </div>
    <div>📈</div>
    <div>
        Dynamics ↔
    </div>
    <div text-align="left">
        <i>RBDL</i>
    </div>
    <div>🦿</div>
    <div>
        Vision ↔
    </div>
    <div text-align="left">
        <i>OpenCv</i>
    </div>
    <div>🥽</div>
    <div>
        Optimal Control ↔
    </div>
    <div text-align="left">
        <i>Biorbd</i>
    </div>
    <div>🕹️</div>
</div>

<Footnotes x='l'>
    <Footnote :number=1>filmfreedonia.com</Footnote>
</Footnotes>

---
image: /bullet_dodge_neo.webp
hideInToc: true
layout: table-of-contents
class: text-left
---
# Table of Contents
<!--
Table of Contents
-->

---
title: Formulation of Optimization Problem 📈
layout: image-right
image: /robot_neo.png
class: text-left
---
# Optimization Problem

> ### Problem:
> - **avoid obstacles** on head level
> - **feet** are static on the ground
>
> $=>$ $\text{const.}$ Polygon of Support
> - **statically stable**
>
> $=>$ $\text{CoP} \in \text{PoS}$

<div class="h-20"></div>

#### Acronyms
Center of Preassure = $\text{CoP}$ \
Polygon of Support = $\text{PoS}$

---
layout: two-cols
class: text-left
hideInToc: true
---
# Optimization <br> Problem

> ### Problem:
> - **avoid obstacles** on head level
> - **feet** are static on the ground
>
> $=>$ $\text{const.}$ Polygon of Support
> - **statically stable**
>
> $=>$ $\text{CoP} \in \text{PoS}$

<div class="h-20"></div>

#### Acronyms
Center of Preassure = $\text{CoP}$ \
Polygon of Support = $\text{PoS}$ \
Center of Mass = $\text{CoM}$

::right::

<div class="text-right">

<div class="h-20"></div>

### Mathematical Formulation $(I)$

<v-switch>
<template #0>
$$
\begin{align}
    p_{\text{head}} &\ne p_{\text{obstacle}} \\
    \nonumber \\
    p_{\text{leftFoot}} &\in \text{GND} \\
    \text{AND}\quad p_{\text{rightFoot}} &\in \text{GND} \\
    \nonumber \\
    p_\text{CoP} &\in \text{PoS} \\
    \xRightarrow{\text{static}} p_\text{CoP} &:= 
    \begin{bmatrix}
        p_\text{CoM}.x \\
        p_\text{CoM}.y \\
        0
    \end{bmatrix}
\end{align}
$$
</template>

<template #1>
$$
\begin{align}
    p_{\text{head}}^\text{world} &\ne p_{\text{obstacle}}^\text{world} \\
    \nonumber \\
    p_{\text{leftFoot}}^\text{world} &\in \text{GND} \\
    \text{AND}\quad p_{\text{rightFoot}}^\text{world} &\in \text{GND} \\
    \nonumber \\
    p_\text{CoP}^\text{world} &\in \text{PoS}^\text{world} \\
    \xRightarrow{\text{static}} p_\text{CoP}^\text{world} &:= 
    \begin{bmatrix}
        p_\text{CoM}^\text{world}.x \\
        p_\text{CoM}^\text{world}.y \\
        0
    \end{bmatrix}
\end{align}
$$
</template>
</v-switch>

</div>

---
title: Stability ⚖
level: 1
---
# Stability
---
title: Center of Mass $(\text{CoM})$ and Center of Preassure $(\text{CoP})$ 🎯
level: 2
---
# Center of Mass $(\text{CoM})$ 🎯

---
hideInToc: true
level: 2
---
# Center of Preassure $(\text{CoP})$

---
level: 2
---
# Polygon of Support $(\text{PoS})$ 👣

---
level: 2
---
# Stability Criteria 📐

---
level: 2
layout: center
---
# <span style="color: blue">Live Demo</span>🥋

---
level: 1
---
# Actuation 🕹

---
level: 1
---
# Executing Optimization ✅

---
level: 2
layout: center
---
# <span style="color: blue">Live Demo</span>🥋
---
level: 1
---
# Future Work 🔭

---
