# TODOs
 Preperation work
- [ ] find suitable name (*projectNeo*)
- [x] add docu for `pkgs` folder in docker repo
- [x] fixed rviz in docker container
- [x] write *com* calculator
- [ ] (optional) fix permissions
- [ ] find joint angles for statically stable matrix pose
- [ ] write package to replicate joint_angles (from echo of topic)
- [ ] write static stability checker
    - [x] calculate support polygon from feet (generating polygon from multiple points)
    - [x] check if center of mass is inside polygon of support (point inside polyon problem)
- [ ] select suitable ground position -> *Karsten*
- [x] check if robot could perform evasion strategies (from joint perspective); can be done in rviz
- [ ] move robot step by step into statically stable position
    - [ ] get example from Karsten running
    - [x] install conda and nlopt, biorbd in docker
    - [ ] use nlopt (optimization) to keep static stability
### Optimal Control
- [ ] write pseudo formulation for optimal control problem
- [ ] build inverse kinematics component with *Optimal Control* framework
- [ ] bring robot in evasion pose with inverse kinematics component
