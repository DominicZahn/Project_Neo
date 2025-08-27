# Optimal Control
> [!info] Change to only estimate optimal evation position
## That shit need to be done
- [x] create masking for joints (`q`)
    - mask rbdlWrapper
    - mask wrapper already in constructor, so model is already masked
- [x] project PoS and CoM to ground
- [x] fix feet to ground
    - new constraint
    - this would change PoS -> *to much effort*
        - feet body needs to be at this world position
        - both feet at same $z$ and $z < 0.0$ (*underneath pelvis)
        - (fix orientation)
    - keep feet at constant postion from pelvis
        - no knees
        - only hip and upper body
- [ ] constraint for dodging
- [ ] fix stability criteria
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
