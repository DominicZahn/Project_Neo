# TODOs
 Preperation work
- [ ] find suitable name (*projectNeo*) -> docker will be called *matrix*
- [x] add docu for `pkgs` folder in docker repo
- [x] fixed rviz in docker container
- [x] write *com* calculator
- [ ] optimize *com* calculator to get faster responses
    - using rbdl to read urdf file
- [ ] (optional) fix permissions
- [ ] find joint angles for statically stable matrix pose
- [x] write static stability checker
    - [x] calculate support polygon from feet (generating polygon from multiple points)
    - [x] check if center of mass is inside polygon of support (point inside polyon problem)
- [ ] select suitable ground position -> *Karsten*
- [x] check if robot could perform evasion strategies (from joint perspective); can be done in rviz
- [ ] move robot step by step while keeping static stability
    - [x] get rbdl and nlopt running with minimal functional example
    - [ ] get Heinz moving
    - [x] install and nlopt in docker and rbdl
    - [ ] calculate inverse kinematic with static stability
        - [ ] formulate optimzation problem (objective function and constraints)
        - [ ] execute and interpolate between poses
### Optimal Control
- [ ] write pseudo formulation for optimal control problem
- [ ] build inverse kinematics component with *Optimal Control* framework
- [ ] bring robot in evasion pose with inverse kinematics component
