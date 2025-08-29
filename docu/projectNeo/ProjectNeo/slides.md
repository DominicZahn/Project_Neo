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
        Optimal Control ↔
    </div>
    <div text-align="left">
        <i>Bioptim</i>
    </div>
    <div>🕹️</div>
    <div>
        Vision ↔
    </div>
    <div text-align="left">
        <i>OpenCv</i>
    </div>
    <div>🥽</div>
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
# Optimization Problem 📈

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
# Optimization <br> Problem  📈

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
level: 1
layout: image-right
image: /R1.jpg
class: "text-center"
---
<div style="display:flex; align-items:center; justify-content:center; height:60vh;">

# Static Stability ⚖

<Footnotes x='l'>
    <Footnote :number=1>Unitree</Footnote>
</Footnotes>

</div>
---
title: Center of Mass $(\text{CoM})$ and Center of Preassure $(\text{CoP})$ 🎯
level: 2
layout: two-cols
---
# Center of Mass <br>$(\text{CoM})$ 🎯
<div class="h-20"></div>

![Repo Card](https://github-readme-stats.vercel.app/api/pin/?username=K-d4wg&repo=ros2_heinz)

::right::
<div class="h-20"></div>
```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    CenterOfMass[CenterOfMass 🎯]

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end
    CenterOfMass -.retrieve urdf.-> h1_description

```

<Footnotes x='l'>
    <Footnote :number=1><a href="https://github.com/K-d4wg/ros2_heinz">K-d4wg</a></Footnote>
</Footnotes>
---
hideInToc: true
level: 2
layout: two-cols
---
# Center of Mass <br>$(\text{CoM})$ 🎯
<div class="grid grid-cols-2 place-items-center gap-y-6">
    <div>
        <img src="/TechStack/ROS.svg" width="150em"/>
    </div>
    <div>
        <img src="/TechStack/rbdl.svg" width="150em"/>
    </div>
    <div>
        <img src="/TechStack/boost.svg" width="130em"/>
    </div>
    <div>
        <img src="/TechStack/eigen.png" width="130em"/>
    </div>
</div>

::right::
<div class="h-20"></div>
```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    CenterOfMass[CenterOfMass 🎯]

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end
    
    subgraph Thirdparty
    subgraph ROS2
        rclcpp
        msgs
        tf2
    end
    RBDL
    Eigen3
    Boost
    end

    CenterOfMass -.-> ROS2
    CenterOfMass -.-> RBDL
    ROS2 -.-> Boost
    RBDL -.-> Eigen3

    CenterOfMass -.-> h1_description

```
<div class="h-10"/>

<v-switch>
<template #1>

> ### 🔊Publisher: `/CoM`

</template>
<template #2>

> ### 🔊Publisher: `/CoM` $\xRightarrow{z=0}$ `/CoP`

</template>
</v-switch>
---
hideInToc: true
level: 2
layout: image-right
image: /screenshots/CoM.png
class: text-right
---

# Center of Mass $(\text{CoM})$ 🎯

<div class="h-20"/>

### Legend

**CoM** 🟠

**CoP** 🔵

---
level: 2
layout: two-cols
---
# Polygon of Support $(\text{PoS})$ 👣
<video
    src="/screenshots/contacts.webm"  autoplay
    width="220em"
    loop
    muted
    playsinline
    class="mx-auto rounded-xl shadow-lg max-h-[100vh]"
/>

::right::
<div class="h-10"/>
```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    subgraph point_calculator
        CenterOfMass[CenterOfMass 🎯]
        PolygonOfSupport[PolygonOfSupport 👣]
    end

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end
    subgraph Thirdparty
    subgraph ROS2
        rclcpp
        msgs
        tf2
    end
    RBDL
    Eigen3
    Boost
    end

    point_calculator -.-> ROS2
    point_calculator -.-> RBDL
    ROS2 -.-> Boost
    RBDL -.-> Eigen3

    point_calculator  -.-> h1_description

```
<div class="h-5"/>

<v-click>

> ### 🔊Publisher: `/PoS`

</v-click>
---
hideInToc: true
level: 2
layout: two-cols
---
# Polygon of Support $(\text{PoS})$ 👣
<video
    src="/screenshots/PoS.webm"  autoplay
    width="220em"
    loop
    muted
    playsinline
    class="mx-auto rounded-xl shadow-lg max-h-[100vh]"
/>

::right::
<div class="h-10"/>
```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    subgraph point_calculator
        CenterOfMass[CenterOfMass 🎯]
        PolygonOfSupport[PolygonOfSupport 👣]
    end

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end
    subgraph Thirdparty
    subgraph ROS2
        rclcpp
        msgs
        tf2
    end
    RBDL
    Eigen3
    Boost
    end

    point_calculator -.-> ROS2
    point_calculator -.-> RBDL
    ROS2 -.-> Boost
    RBDL -.-> Eigen3

    point_calculator  -.-> h1_description

```
<div class="h-5"/>

> ### 🔊Publisher: `/PoS` $\xRightarrow{\text{Rviz}}$ `/vis_PoS`

---
#hideInToc: true
level: 3
layout: image-right
image: /screenshots/gz_faceplant.png
class: text-right
transition: fade
---

<div class="h-10"/>

# Gazebo $\xRightarrow{}$ Rviz

<v-clicks>

### Flexibel Joint Control ✅
### Full Visualization ✅
### No Jiggeling ✅
### Better Performance ✅

</v-clicks>
<div class="h-10"/>
<v-clicks>

### No Physics ❌
### No Fixed Global Frame ❌
### No Live Contacts ❌

</v-clicks>
<div class="h-10"/>
<v-click>

## $\xRightarrow{}$ Fixed $\text{PoS}$ 🔒

</v-click>
---
level: 2
layout: two-cols
class: text-left
---

# Stability Criteria 📐

<img class="mx-auto block" width="80%" src="/screenshots/minEdgeDst.png"/>
<div class="text-center">

## Min Edge Distance
#### 🚫 No Continuity
#### ✅ Uses PoS Borders
#### ✅ Adjusts to Shape

</div>

::right::
<div class="text-right h-14">
🔴: low values | 🔵: high values
</div>

<img class="mx-auto block" width="80%" src="/screenshots/centroidDst.png"/>
<div class="text-center">
<span v-mark="{color: 'green', type: 'circle' }">

## Centroid Distance
</span>

#### ✅ Continuious
#### 🚫 Ignores PoS Borders
#### ⚠ Only Indirect Influence of Shape
</div>

---
level: 3
layout: two-cols
hideInToc: true
---

# Stability ⚖
<div class="h-20"/>
<h2>📚Stability Library</h2>
<h4><b>- Stability Criteria</b></h4>
<h4><b>- Helper Functions</b></h4>

<div class="h-10"/>
<h2>🔗Depends on</h2>
<h4><b> - RBDL</b></h4>
<h4><b> - h1_description</b></h4>

::right::
<div class="h-20"/>

```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    subgraph point_calculator
        CenterOfMass[CenterOfMass 🎯]
        PolygonOfSupport[PolygonOfSupport 👣]
    end

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end
    subgraph Thirdparty
    subgraph ROS2
        rclcpp
        msgs
        tf2
    end
    RBDL
    Eigen3
    Boost
    end

    point_calculator -.-> ROS2
    point_calculator -.-> RBDL
    ROS2 -.-> Boost
    RBDL -.-> Eigen3

    point_calculator  -.-> h1_description

    Stability[Stability ⚖]
    style Stability stroke-dasharray: 5 5

    Stability -.-> RBDL
    Stability -.-> h1_description

    PolygonOfSupport -.-> Stability
```
---
level: 2
layout: two-cols
hideInToc: true
---

# Launch Files

<div class="h-30"/>
<h2> 🚀 stability.launch</h2>
<h4><i>run</i> <b>CenterOfMass 🎯</b></h4>
<h4><i>run</i> <b>PolygonOfSupport 👣</b></h4>
<h4><i>launch</i> <b>rviz</b> <i>OR</i> <b>gazebo🧮</b></h4>

::right::
<div class="h-5"/>

```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    subgraph point_calculator
        CenterOfMass[CenterOfMass 🎯]
        PolygonOfSupport[PolygonOfSupport 👣]
    end

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end
    subgraph Thirdparty
    subgraph ROS2
        rclcpp
        msgs
        tf2
    end
    RBDL
    Eigen3
    Boost
    end

    point_calculator -.-> ROS2
    point_calculator -.-> RBDL
    ROS2 -.-> Boost
    RBDL -.-> Eigen3

    point_calculator  -.-> h1_description

    Stability[Stability ⚖]
    style Stability stroke-dasharray: 5 5

    Stability -.-> RBDL
    Stability -.-> h1_description
    PolygonOfSupport -.-> Stability

    stability.launch[stability.launch 🚀]

    stability.launch --launch--> h1_bringup
    stability.launch --run--> CenterOfMass
    stability.launch --run--> PolygonOfSupport
```

---
level: 1
layout: two-cols
---
# Actuation 🕹
<div class="h-30"/>
<h2>🔊Publishing to <i>/joint_states</i></h2>
<h4>- 🍎<b>Ignore Physics</b></h4>
<h4>- ⏱<b>Instant Movement</b></h4>

::right::
<video
    src="/screenshots/moving.webm" autoplay
    width="500em"
    loop
    muted
    playsinline
    class="mx-auto rounded-xl shadow-lg max-h-[100vh]"
/>

---
level: 2
layout: two-cols
hideInToc: true
---

# Dependencies 🔗

<div class="h-30"/>
<v-click>
    <img class="mx-auto block" width="100em" src="/TechStack/cmake.svg"/>
    <h2 class="text-center">🚨 <b>Dependency Chaos</b> 🚨</h2>
</v-click>

::right::
<div class="h-5"/>

```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    subgraph point_calculator
        CenterOfMass[CenterOfMass 🎯]
        PolygonOfSupport[PolygonOfSupport 👣]
    end

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end
    subgraph Thirdparty
    subgraph ROS2
        rclcpp
        msgs
        tf2
    end
    RBDL
    Eigen3
    Boost
    end

    point_calculator -.-> ROS2
    point_calculator -.-> RBDL
    ROS2 -.-> Boost
    RBDL -.-> Eigen3

    point_calculator  -.-> h1_description

    Stability[Stability ⚖]
    style Stability stroke-dasharray: 5 5

    Stability -.-> RBDL
    Stability -.-> h1_description
    PolygonOfSupport -.-> Stability

    stability.launch[stability.launch 🚀]

    stability.launch --launch--> h1_bringup
    stability.launch --run--> CenterOfMass
    stability.launch --run--> PolygonOfSupport
```

---
level: 2
layout: two-cols
class: text-center
---
<div class="text-left">

# RbdlWrapper 🛠
</div>
```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    subgraph point_calculator
        CenterOfMass[CenterOfMass 🎯]
        PolygonOfSupport[PolygonOfSupport 👣]
    end

    stability.launch[stability.launch 🚀]

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end

    subgraph neo_utils
        RbdlWrapper[RbdlWrapper 🛠]
        Stability[Stability ⚖]
        style RbdlWrapper stroke-dasharray: 5 5
        style Stability stroke-dasharray: 5 5
    end

    point_calculator -.-> neo_utils

    dodge_it -.-> RbdlWrapper

    stability.launch --launch--> h1_bringup
    stability.launch --run--> CenterOfMass
    stability.launch --run--> PolygonOfSupport

    RbdlWrapper -.-> h1_description

```

::right::

<v-switch>
<template #0>
```mermaid
flowchart BT
    User --access--> RBDL
    subgraph RbdlWrapper:
        RBDL <--sync--> js[joint_states]
        end
```
</template>
<template #1>
```mermaid
flowchart BT
    User .-> RBDL
    User --access--> Mask
    subgraph RbdlWrapper:
        Mask <--> RBDL
        RBDL <--sync--> js[joint_states]
        end
```
</template>
</v-switch>


---
level: 1
hideInToc: true
layout: center
class: text-left
---
<div class="text-left">

# Framework Overview 🧰

</div>

```mermaid
%%{
  init: {
    'theme': 'neutral'
  }
}%%

flowchart TB
    subgraph point_calculator
        CenterOfMass[CenterOfMass 🎯]
        PolygonOfSupport[PolygonOfSupport 👣]
    end

    subgraph dodge_it
        Smith[Smith 🤺]
        Neo[Neo 🤸]
        stability.launch[stability.launch 🚀]
    end

    subgraph ros2_heinz[ros2_heinz *1*]
        subgraph h1_gazebo_sim
            subgraph h1_bringup
                h1_rviz.launch
                h1_gazebo_sim.launch
            end
            subgraph h1_description
            end
        end
    end

    subgraph neo_utils
        RbdlWrapper[RbdlWrapper 🛠]
        Stability[Stability ⚖]
        style RbdlWrapper stroke-dasharray: 5 5
        style Stability stroke-dasharray: 5 5
    end

    point_calculator -.-> neo_utils
    
    Smith -.-> neo_utils
    Neo -.-> neo_utils

    stability.launch --launch--> h1_bringup
    stability.launch --run--> CenterOfMass
    stability.launch --run--> PolygonOfSupport

    RbdlWrapper -.-> h1_description

```

<Footnotes x='l'>
    <Footnote :number=1><a href="https://github.com/K-d4wg/ros2_heinz">K-d4wg</a></Footnote>
</Footnotes>
---
level: 2
layout: center
class text-center
---
# Tech Stack Overview 📦
<div class="h-5"/>
<div class="grid grid-cols-4 gap-x-25 gap-y-10">
  <div><img class="mx-auto block" width="100em" src="/TechStack/ROS.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/Docker.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/gz.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/rviz.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/rbdl.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/nlopt.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/boost.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/eigen.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/biorbd.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/bioviz.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/bioptim.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/OpenCV.svg"/></div>
</div>
---
level: 1
layout: image-right
image: /TechStack/nlopt_exp.png
class: text-center
---

<div style="display:flex; align-items:center; justify-content:center; height:60vh;">

# Executing Optimization 🎲

</div>
---
hideInToc: true
level: 2
layout: image-right
image: /screenshots/well_thats_an_optimum.png
---
# Root $\neq$ World

<div class="text-right">

### Optimization **(I)**
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
</div>
---
hideInToc: true
level: 2
layout: image-right
image: /screenshots/dodging_literally_everything.png
---
# Root $\neq$ World

<div class="text-right">

### Optimization **(II)**
$$
\begin{align}
    p_{\text{head}}^\text{leftFoot} &\ne p_{\text{obstacle}}^\text{leftFoot} \\
    \nonumber \\
    p_{\text{leftFoot}}^\text{leftFoot} &\in \text{GND} \\
    \text{AND}\quad p_{\text{rightFoot}}^\text{leftFoot} &\in \text{GND} \\
    \nonumber \\
    p_\text{CoP}^\text{leftFoot} &\in \text{PoS}^\text{leftFoot} \\
    \xRightarrow{\text{static}} p_\text{CoP}^\text{leftFoot} &:= 
    \begin{bmatrix}
        p_\text{CoM}^\text{leftFoot}.x \\
        p_\text{CoM}^\text{leftFoot}.y \\
        0
    \end{bmatrix}
\end{align}
$$
</div>
---
hideInToc: true
level: 2
layout: image-right
image: /screenshots/dodging_literally_everything.png
class: text-left
---
<div class="text-center">
```mermaid
flowchart BT
    User .-> RBDL
    User --access--> Mask
    subgraph RbdlWrapper:
        Mask <--> RBDL
        RBDL <--sync--> js[joint_states]
        end
    style Mask fill:orange, color:grey
```
</div>
---
hideInToc: true
level: 2
layout: figure
figureUrl: /screenshots/face_down.png
figureCaption: 🔗 unsynced Legs + 🎱 bad Start Configuration
---
# Just a Little Push 🥊

---
level: 2
layout: image-right
image: /dojo_render.jpg
class: text-center
---

<div style="display:flex; align-items:center; justify-content:center; height:60vh;">

# <span style="color: green"> 🥋 <b>Live Demo</b> 🥋 </span>

</div>
<Footnotes x='l'>
    <Footnote :number=1>learnvray.com</Footnote>
</Footnotes>
---
hideInToc: true
level: 2
layout: figure
figureUrl: /screenshots/finally.png
class: text-center
---
# 🎉Welcome `He1nz` the chosen one🎉
---
level: 1
---
# Future Work 🔭

```mermaid
%%{
  init: {
    'theme': 'base',
    'themeVariables': {
      'primaryColor': '#2596be',
      'primaryTextColor': 'white',
      'primaryBorderColor': '#7C0000',
      'secondaryColor': '#b04d1e',
      'tertiaryColor': '#bea925'
    }
  }
}%%
mindmap
    root((Project Neo))
        Optimal Control
            Stable Trajectories
            Dynamic Stability
            Full Gazebo Simulation
        Dynamic PoS
            Realtime Contacts
            Adaptable PoS
        Object Detection
            Simulate Obstacle
            Add LiDar and Camera preprocessing
            Explore Evasion Constraints
```

---
hideInToc: true
layout: end
---
# Thank you for following the White Rabbit. 🐇
---
