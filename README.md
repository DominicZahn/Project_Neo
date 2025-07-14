# Whole Body-LAB
A repository holding all code regarding "Whole Body Locomotion".

### Setup Process
1. git clone with `--recursive`
2. execute `make build` ->
    - creats `ws/src/`
    - links packages from `h1_2_pkgs` to `h1_2/ws/src/`

### TODOs
- [x] add docu for `pkgs` folder in docker repo
- [ ] check if robot could perform evasion strategies (from joint perspective)
- [ ] write *com* calculator
- [ ] decide on *Optimal Control* framework
    - [ ] cpp or python
    - [ ] build inverse kinematics component with *Optimal Control* framework
    - [ ] bring robot in evasion pose with inverse kinematics component
