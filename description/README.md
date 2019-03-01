# `description`

Contains robot description files (URDFs and kindsl files).

The files are generated from the source URDFs in `urdf/source` and the accompanying `*_floating` files by running

```julia
julia generation/generate_description_files.jl
```

See this Julia script for details on how the robot description files are generated.

The source URDFs were obtained as follows:

* Atlas: https://raw.githubusercontent.com/RobotLocomotion/drake/73a8da32cd41ff7fd023c3680f8250860cbd0e6b/examples/atlas/urdf/atlas_convex_hull.urdf, modified from a version supplied by OSRF as part of the DRC sim
* HyQ: https://github.com/iit-DLSLab/hyq-description/blob/43cc075a628a41a30779cfde22f249aef3ae2d67/robots/hyq_no_sensors.urdf.xacro, converted to URDF using xacro (https://wiki.ros.org/xacro), `rosrun xacro xacro robots/hyq_no_sensors.urdf.xacro > hyq_no_sensors.urdf`
* Kuka IIWA: https://github.com/bulletphysics/bullet3/blob/180a9f5103dadec7edc7beed6d8cde0402ad8c22/data/kuka_iiwa/model.urdf
