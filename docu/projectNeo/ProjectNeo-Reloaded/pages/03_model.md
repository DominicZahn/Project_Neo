---
title: Model Intro
layout: image-right
image: /inertia.png
class: text-right
---

<div class="h-50"/>

# How to Model `H`e`1`nz? 🧬

---
title: Lumped Model
layout: image-right
image: /frontal_gz.png
backgroundSize: contain
class: text-right
---

<div class="h-5"/>

# Lumped<br>Model 🤸

```mermaid { scale: 0.65 }
flowchart TD
  ub("<font color="#b100ff"><b>Upper Body</b></font>"):::big
  style ub stroke:#b100ff
  t("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color="#0093ff"><b>Thigh</b></font>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"):::big
  style t stroke:#0093ff
  s("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color="#bad181"><b>Shank</b></font>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"):::big
  style s stroke:#bad181
  f("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color="#ff6a00"><b>Foot</b></font>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"):::big
  style f stroke:#ff6a00

  classDef big font-size:35px,fill:white,stroke-width:3

  ub --"<i>hip</i>"--> t --"<i>knee</i>"--> s --"<i>ankle</i>"--> f
```

---
title: Lumped Model
layout: image-right
image: /side_gz.png
backgroundSize: contain
class: text-right
---

<div class="h-5"/>

# Mirrored 🪞 <br> Pitch Joints
```mermaid { scale: 0.65 }
flowchart TD
  ub("<font color="#b100ff"><b>Upper Body</b></font>"):::big
  style ub stroke:#b100ff
  t("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color="#0093ff"><b>Thigh</b></font>🔗"):::big
  style t stroke:#0093ff
  s("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color="#bad181"><b>Shank</b></font>🔗"):::big
  style s stroke:#bad181
  f("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<font color="#ff6a00"><b>Foot</b></font>🔗"):::big
  style f stroke:#ff6a00

  classDef big font-size:35px,fill:white,stroke-width:3

  ub --"<i>hip</i>"--> t --"<i>knee</i>"--> s --"<i>ankle</i>"--> f
```

---
title: Mirror Layer
class: text-center
---

<div class="h-30"/>
<h1 class="text-center">Reduction Layer</h1>
<div class="h-5"/>

```mermaid
flowchart LR
  classDef body fill:white
  classDef none stroke:white,fill:white

    p["<wbr><img src="/TechStack/pinocchio_logo.png"/>Rigid Body Dynamics"]
    c["<wbr><img src="/TechStack/casadi.png"/>Computer Algebra"]
    a["<wbr><img src="/TechStack/acados_logo.png"/>Optimal Control"]
    m["<h1><wbr>🔗+🪞</h1> <br>Placeholder_____<br>T"]

    style m fill:white,stroke-dasharray:40 20,stroke-width:3

    p:::body --- c:::none --> m:::body --> a:::body
```
