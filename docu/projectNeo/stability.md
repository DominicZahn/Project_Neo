# Stability
## Static Stability
- based on PoS (Polygon of Support) and CoM (Center of Mass) projected to ground
> [!info Regularisation]
> Adding a small weighted metric to the configuration to differentiate between joint configurations with the same stability score.
- static stability criteria needs to be continous (because of optimal control)
### Distance to Centroid
- does not take long PoS bias into account
- faster in computation
### SUM Distance to Points of PoS
- continous
- stable even if PoS changes
### SUM Distance to Segments of PoS
- not suitable
- is always the same
> -> constant value!!!
### MIN Distance to Segments of PoS
- not continous
- sometimes jumpy if the PoS changes
