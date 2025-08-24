# Optimal Control
> [!info] Change to only estimate optimal evation position
## That shit need to be done
- [ ] create masking for joints (`q`)
- [ ] fix feet to ground
    - new constraint
    - feet body needs to be at this world position
- [x] only rviz
    - [x] set joint position
    - [x] visualize CoM and saved PoS
## Second Prio i guess
- [x] move launch files from `neo_utils` to `dodge_it`
- [ ] contact contraints for feet
- [ ] passiv option for *rbdlWrapper* (**is already passiv?**)
- [ ] read in PoS dynamic on startup
- [ ] find better constraint than head should not be here
- [ ] create visualization for *bullet*
- [x] combine everything in one run (no need to restart or put poses by hand into rviz)
