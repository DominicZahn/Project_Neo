# TODOs
- [x] find suitable name (*projectNeo*)
        -> docker is **matrix**
        -> different stages have names of different characters
        -> **posponted to presentation prep**
- [x] add docu for `pkgs` folder in docker repo
- [x] fixed rviz in docker container
- [x] write *com* calculator
- [x] optimize *com* calculator to get faster responses
    - using rbdl to read urdf file
- [ ] (optional) fix permissions
- [ ] add parameter to launch file to activate gazebo (else rviz)
- [x] write static stability checker
    - [x] calculate support polygon from feet (generating polygon from multiple points)
    - [x] check if center of mass is inside polygon of support (point inside polyon problem)
- [x] check if robot could perform evasion strategies (from joint perspective); can be done in rviz
~- [ ] move robot step by step while keeping static stability~
    - [x] get rbdl and nlopt running with minimal functional example
    - [x] get Heinz moving
    - [x] install and nlopt in docker and rbdl
    - [x] read position from txt file
    - [x] move robot to that postion and stop if stability is insufficient
    - [x] formulate optimization problem
    ~- [ ] use optimization to find new joint configurations and move slowly towards it~

    - [ ] fix to small increment problem (**if thats even possible**)
    -> use integer arithmetic
### Stability Criteria
- [x] visualize different criteria (heatmap of a given PoS)
- [x] fix PoS
    - [x] make more stable over time -> *more points more stable*
### Dodge Control
> Using rbdl and nlopt to let h1 dodge away from a given postion at a given point in time.
- [ ] write pseudo formulation for optimal control problem
- [ ] spawn robot in pose
- [ ] visualize results of optimization

# Q&A
- why incrementation problem????? -> integer arithmetic, multiplie of low double
## New Project Target
> Focus on the dodgin part!
- find opitimal position for given position avoidance
- definetly check if target position is stable -> for optimal control
- outlook
    - dynamic stability better
    - optimal control better instead of step by step
