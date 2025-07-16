# Whole Body-LAB
A repository holding all code regarding "Whole Body Locomotion".

### Setup Process
1. git clone with `--recursive`
2. execute `make build` ->
    - creats `ws/src/` and binds to current workspace
    - links packages from `pkgs` to `h1_2/ws/pkgs/`
    - some other docker stuff

### TODOs
- [x] add docu for `pkgs` folder in docker repo
- [x] fixed rviz in docker container
- [ ] write pseudo formulation for optimal control problem
- [ ] check if robot could perform evasion strategies (from joint perspective); can be done in rviz
- [ ] write *com* calculator
- [ ] decide on *Optimal Control* framework
    - [x] cpp or python -> **both**
    - [ ] build inverse kinematics component with *Optimal Control* framework
    - [ ] bring robot in evasion pose with inverse kinematics component

### Optimal Control
- equality of momenten
- objective function: 
    - maintain stability: high stability metric
- constraint: moving out of collision
- capture point
- frame by frame statically stable
    - move to position slowly
