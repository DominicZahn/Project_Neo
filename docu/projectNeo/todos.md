# TODOs
 Preperation work
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
- [x] write static stability checker
    - [x] calculate support polygon from feet (generating polygon from multiple points)
    - [x] check if center of mass is inside polygon of support (point inside polyon problem)
- [x] check if robot could perform evasion strategies (from joint perspective); can be done in rviz
- [ ] move robot step by step while keeping static stability
    -> *we need nlopt*
    - [x] get rbdl and nlopt running with minimal functional example
    - [x] get Heinz moving
    - [x] install and nlopt in docker and rbdl
    - [x] read position from txt file
    - [x] move robot to that postion and stop if stability is insufficient
    - [x] formulate optimization problem
    - [ ] use optimization to find new joint configurations and move slowly towards it
    - [ ] update static stability criteria

    - [ ] fix to small increment problem (**if thats even possible**)
### Stability Criteria
- [ ] visualize different criteria (heatmap of a given PoS)
- [x] fix PoS
    - [x] make more stable over time -> *more points more stable*
    - [ ] project to ground CoM and PoS -> *to test*
### Optimal Control
- [ ] write pseudo formulation for optimal control problem
- [ ] build inverse kinematics component with *Optimal Control* framework
- [ ] bring robot in evasion pose with inverse kinematics component
