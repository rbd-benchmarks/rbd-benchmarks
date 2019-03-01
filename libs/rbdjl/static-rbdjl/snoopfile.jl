include(joinpath(@__DIR__, "staticrbd.jl"))
using .StaticRBD
using RigidBodyDynamics
using LinearAlgebra

BLAS.set_num_threads(1)
GC.enable(true)

# repeated info from BenchmarkInputDataGen module, but don't really want to depend on that here.

struct URDFConfig
    basename::String
    floating::Bool
end

const URDF_DIR = joinpath(@__DIR__, "..", "..", "..", "description", "urdf")
const URDF_CONFIGS = [URDFConfig("iiwa.urdf", false), URDFConfig("atlas.urdf", true), URDFConfig("hyq.urdf", true)]
const CSV_DIR = joinpath(@__DIR__, "..", "..", "csv", "rbdjl")

for config in URDF_CONFIGS
    urdf = joinpath(URDF_DIR, config.basename)
    robotname, _ = splitext(config.basename)
    csv = joinpath(CSV_DIR, "rbdjl", robotname, ".csv")
    floating = config.floating

    c_floating = Cint(floating)
    for c_scalar_type in [Cint(1), Cint(2)]
        mechanism = GC.@preserve urdf csv begin
            c_urdf = Base.unsafe_convert(Cstring, urdf)
            c_csv = Base.unsafe_convert(Cstring, csv)
            StaticRBD.create_mechanism(c_urdf, c_floating, c_scalar_type)
        end
        state = StaticRBD.create_state(mechanism)
        result = StaticRBD.create_dynamics_result(mechanism)

        jointwrenches = result.jointwrenches
        accelerations = result.accelerations
        M = result.massmatrix
        vd = result.v̇

        q = configuration(state)
        v = velocity(state)
        vd_desired = similar(v)
        tau = similar(v)

        StaticRBD.inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired)
        StaticRBD.mass_matrix(M, state)
        StaticRBD.dynamics(result, state, tau)
    end
end
