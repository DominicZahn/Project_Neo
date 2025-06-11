# Exercise 3
**Prof. Dr. Katja Mombaur**

**Humanoid Robotics ‑ Locomotion and Whole‑Body Control (SS 2025)**

**Exercise Sheet No. 3**

**Due: June 23, 2024**

---
| | done | working |
|-|:-:|:-:|
| :heavy_check_mark: | **X** | **X** |
| :heavy_minus_sign: | **X** |  | 
| :heavy_multiplication_x: |  |  |

### 0.0 General Questions

### 3.0 - Setup H1
#### 3.0.1 - Custom Dev-Container
| Info |  |
|-|-|
| **OS** | Ubunutu 22.04 |
| **ROS** | humble |

##### Additional Software / Packages
- nvidia stuff
- btop
- tmux
- xserver ?
#### 3.0.2 - Get H1 up and running

### 3.1 – Support Polygon

Figure 1 shows walking patterns of persons walking with crutches in the z‑y‑plane.  
Include (i.e. draw) all the support polygons into this figure for the times at which both feet as well as both crutches (next to the body or in front of the body) are in contact with the ground.

For drawing the polygons of support, consider heel and toe points of the feet (marked by circles and crosses) and the crutch positions (marked by triangles).

Complete figure 1 with the correctly drawn support polygons and prepare a picture for presentation at the front.

> **Figure 1**: Walking with crutches (foot patterns marked by heel and toe points, triangles denote crutch placement)

---

### 3.2 – 3D LIPM Pattern generation

Implement the algorithm discussed in the lecture and given below (Algorithm for walking pattern generation with 3D LIP) in a language of your choice.

Colored numbers refer to the respective equations in the slides.  
Feel free to use another sign convention for the step parameters as discussed in the class.

Apply the algorithm to solve the following problems:

- straight walking motion as shown at the end of the lecture on the 6.2.2025 with 3 steps + starting and stopping.
- a sideward walking motion moving 25 cm to the right at each right step and setting the left foot at 10 cm distance to it, with a total of 6 steps.
- another walking motion of your choice, you are encouraged to be creative.

Prepare figures or live demonstrations for sufficient visualization of the results in all cases, the corresponding step parameter tables for all cases, and all relevant code.

> For additional reading, consider consulting the provided reference:  
> **Kajita et al.: Introduction to Humanoid Robots**

---

