# Polygon of Support (PoS)
## Building and Updating Polygon
### Rebuild on every new iteration
- build polygon from all points (no check if points are inside polygon)

### Update Polygon over time
**TBD**

## Check CoM
-> use `boost::geometry::covered_by`


> We just use `boost` to build a polygon and do the inside check!
> So everything that follows is not necessary anymore.
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
