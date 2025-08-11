# TODOs
 Preperation work
- [ ] find suitable name (*projectNeo*)
        -> docker is **matrix**
        -> different stages have names of different characters
- [x] add docu for `pkgs` folder in docker repo
- [x] fixed rviz in docker container
- [x] write *com* calculator
- [ ] optimize *com* calculator to get faster responses
    - using rbdl to read urdf file
- [ ] (optional) fix permissions
- [x] write static stability checker
    - [x] calculate support polygon from feet (generating polygon from multiple points)
    - [x] check if center of mass is inside polygon of support (point inside polyon problem)
- [ ] select suitable ground position -> *Karsten*
- [x] check if robot could perform evasion strategies (from joint perspective); can be done in rviz
- [ ] move robot step by step while keeping static stability
    - [x] get rbdl and nlopt running with minimal functional example
    - [x] get Heinz moving
    - [x] install and nlopt in docker and rbdl
    - [ ] read position from yaml file
    - [ ] move robot to that postion and stop if stability is insufficient
    - [ ] *check differnt poses and there path to it*
    - [ ] ?*move from pose to pose*?
    - [ ] *find out what to do with stability next*
### Optimal Control
- [ ] write pseudo formulation for optimal control problem
- [ ] build inverse kinematics component with *Optimal Control* framework
- [ ] bring robot in evasion pose with inverse kinematics component
