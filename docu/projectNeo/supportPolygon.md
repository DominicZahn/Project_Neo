# Polygon of Support (PoS)
## Basic Algorithms for Point in Polygon (PiP) Problem
### Ray crossing alorithm with even-odd-rule
1. Shooting ray from point in any direction
2. Check how many edges are intersected (check tests all edges)
3. even: inside; odd: outside

#### How to Raycross?
##### Boost
- we already use boost
-> boost it is
##### Straight equations with segments (own)
- use straight equations
- intersection as straight coordinates of edge straight (single scalar)
    - must be between striaght coordinates of corners

## Construction
- constucting a polygon from multiple points
- points should have a lifetime (small)
- check if new point is inside polygon -> add to polygon if outside

## Center of Mass (CoM) inside Polygon of Support (PoS)
- use PiP algorithms
