# Input data generation

`Makefile` targets:

* `all` / `csv`: generate a `csv` subdirectory with library-specific `.csv` files.
* `clean`: remove the `csv` directory.
* `test`: run tests.

`Makefile` variables:

* `JULIA` (default: `julia`)

## Notes

Since RobCoGen does not support setting the configuration of a floating body, we opted to set the gravitational
acceleration to zero for forward/inverse dynamics (the mass matrix is independent of gravity anyway). This is because
the gravitational acceleration is the only thing that makes the dynamics depend on the configuration of the floating body.

RobCoGen requires special treatment for robots with a floating base:

* the `q` part of the CSV inputs only contains the configuration of the non-floating joints and should correspond exactly to a `JointState`
* the first six elements of the `v` part of the CSV inputs should be converted to a `Velocity` (i.e., a 6D `Eigen` vector). The remaining part of `v` should be passed in as `qd` (a `JointState`). Similarly for `vd` (`qdd`).
* the first six elements of the `tau` part of the CSV should be omitted. The remaining part should be passed in as the `tau` argument to `fd` (a `JointState`).
