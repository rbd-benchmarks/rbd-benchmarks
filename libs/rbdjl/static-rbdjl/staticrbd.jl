import Pkg
Pkg.activate(@__DIR__)

module StaticRBD

using RigidBodyDynamics
using DelimitedFiles
using LinearAlgebra

Base.@ccallable function create_mechanism(c_urdf::Cstring, c_floating::Cint, c_scalar_type::Cint)::Any
    T = Dict(Cint(1) => Float64, Cint(2) => Float32)[c_scalar_type]
    urdf = unsafe_string(c_urdf)
    floating = Bool(c_floating)
    root_joint_type = floating ? QuaternionFloating{T}() : Fixed{T}()
    mechanism = parse_urdf(urdf; scalar_type=T, root_joint_type=root_joint_type)
    remove_fixed_tree_joints!(mechanism)
    mechanism.gravitational_acceleration = zero(mechanism.gravitational_acceleration)
    mechanism
end

Base.@ccallable function create_state(mechanism::Any)::Any
    MechanismState(mechanism)
end

Base.@ccallable function create_dynamics_result(mechanism::Any)::Any
    DynamicsResult(mechanism)
end

Base.@ccallable function inverse_dynamics(τ::Any, jointwrenches::Any, accelerations::Any, state::Any, v̇::Any)::Cvoid
    setdirty!(state)
    inverse_dynamics!(τ, jointwrenches, accelerations, state, v̇)
    nothing
end

Base.@ccallable function mass_matrix(M::Any, state::Any)::Cvoid
    setdirty!(state)
    mass_matrix!(M, state)
    nothing
end

Base.@ccallable function dynamics(result::Any, state::Any, τ::Any)::Cvoid
    setdirty!(state)
    dynamics!(result, state, τ)
    nothing
end

end
