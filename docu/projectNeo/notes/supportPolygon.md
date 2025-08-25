# Polygon of Support (PoS)
## Building and Updating Polygon
### Rebuild on every new iteration
- build polygon from all points (no check if points are inside polygon)

### Update Polygon over time
- no initalization with points
- points are gathered with subscriptions directly inside the Object
- callbacks are filling each a buffer
- callbacks link to polygon construction after
- Object also publishes
> not sure if it is elegant to put all of this into one class

## Check CoM
-> use `boost::geometry::covered_by`

## Visualize in Rviz
1. triangulate polygon
    1.1. calculate centroid
    1.2. centroid and edge create a triangle
2. create triangle mesh
    - `visualization_msgs::msg::Marker`
    - `marker.type = 11` (TRIANGLE_LIST)
    - .points (contains triangle points 0-1-2, 3-4-5, ...)
    - .pose and .scale transform .points
3. publish

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
