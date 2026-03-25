---
title: Future Work
class: text-left
---

# 🔭 Future Work

<div class="h-15"></div>

<h2 class="h-15"> 🤸 Other Dynamic Stability Criterias</h2>
<h2 class="h-15"> ⏲️Faster Motion</h2>
<h2 v-click="1" class="h-15"> 🦾 Sim2Real (real <b>H</b>e<b>1</b>nz)</h2>
<h2 v-click="1" class="h-25"> 🧙‍♂️ Model Predictive Control</h2>

<h2 v-click="2" class="h-15"> 🔫 Active Dodging</h2>

---
title:
class: text-center
layout: image-right
image: inertia.png
---

# Ideas for Active Dodging 🔫

<div class="h-15"/>

### Constraint 🚧
$$
\begin{align}
p_{obs}(t) \notin \text{conv } R(t) \ \forall t \in [0, T_f] \nonumber \\
\end{align}
$$

<div class="h-15"/>

### Lagrange Term 💶
$$
\begin{align}
\frac{1}{d(\text{conv } R(t),\ p_{obs}(t))} \ \forall t \in [0, T_f] \nonumber \\
\end{align}
$$
