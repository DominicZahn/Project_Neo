---
title: Previous Tech Stack
layout: center
class text-center
---

# <font color="sky">Previous</font> Tech Stack 📦

<div class="h-5"/>
<div class="grid items-center grid-cols-4 gap-x-25 gap-y-10">
  <div><img class="mx-auto block" width="100em" src="/TechStack/ROS.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/Docker.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/gz.svg"/></div>
  <div v-mark="{ at: 1, color: 'oklch(68.5% 0.169 237.323)', type: 'box'}"><img class="mx-auto block" width="100em" src="/TechStack/rviz.png"/></div>
  <div v-mark="{ at: 1, color: 'oklch(68.5% 0.169 237.323)', type: 'box'}"><img class="mx-auto block" width="100em" src="/TechStack/rbdl.svg"/></div>
  <div v-mark="{ at: 1, color: 'oklch(68.5% 0.169 237.323)', type: 'box'}"><img class="mx-auto block" width="100em" src="/TechStack/nlopt.png"/></div>
  <div v-mark="{ at: 1, color: 'oklch(68.5% 0.169 237.323)', type: 'box'}"><img class="mx-auto block" width="100em" src="/TechStack/boost.svg"/></div>
  <div v-mark="{ at: 1, color: 'oklch(68.5% 0.169 237.323)', type: 'box'}"><img class="mx-auto block" width="100em" src="/TechStack/eigen.png"/></div>
  <div v-mark="{ at: 2, color: 'red', type: 'crossed-off' }"><img class="mx-auto block" width="100em" src="/TechStack/biorbd.png"/></div>
  <div v-mark="{ at: 2, color: 'red', type: 'crossed-off' }"><img class="mx-auto block" width="100em" src="/TechStack/bioviz.png"/></div>
  <div v-mark="{ at: 2, color: 'red', type: 'crossed-off' }"><img class="mx-auto block" width="100em" src="/TechStack/bioptim.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/OpenCV.svg"/></div>
</div>

---
title: Current Tech Stack
level: 2
layout: center
class: text-center
---

# <font color="purple">Updated Control</font> Tech Stack 📦

<div class="h-5"/>
<div class="grid items-center grid-cols-4 gap-x-15 gap-y-10">
  <div><img class="mx-auto block" width="100em" src="/TechStack/ROS.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/Docker.svg"/></div>
  <div v-mark="{ at: 1, color: 'purple', type: 'box' }"><img class="mx-auto block" width="100em" src="/TechStack/gz.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/rviz.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/rbdl.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/nlopt.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/boost.svg"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/eigen.png"/></div>
  <div v-mark="{ at: 1, color: 'purple', type: 'box' }"><img class="mx-auto block" width="200em" src="/TechStack/acados_logo.png"/></div>
  <div v-mark="{ at: 1, color: 'purple', type: 'box' }"><img class="mx-auto block" width="200em" src="/TechStack/casadi.png"/></div>
  <div v-mark="{ at: 1, color: 'purple', type: 'box' }"><img class="mx-auto block" width="200em" src="/TechStack/pinocchio_logo.png"/></div>
  <div><img class="mx-auto block" width="100em" src="/TechStack/OpenCV.svg"/></div>
</div>

---
title: Tech Pipeline
class: text-center
---

<div class="h-20"/>

# Pipeline 🏭

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
  classDef none stroke:white,fill:white

  subgraph sub [ ]
    direction LR

    p["<wbr><img src="/TechStack/pinocchio_logo.png"/>Rigid Body Dynamics"]
    c["<wbr><img src="/TechStack/casadi.png"/>Computer Algebra"]
    a["<wbr><img src="/TechStack/acados_logo.png"/>Optimal Control"]

     p:::body --- c:::none -->a:::body
  end
  style sub fill:white,stroke-dasharray:40 20,stroke-width:5,stroke:black
```
  </template>

</v-switch>

<div v-click="1" class="absolute right-0.7em top-8.5em">
  <img src="/TechStack/gz.svg" width="100em"/>
</div>
