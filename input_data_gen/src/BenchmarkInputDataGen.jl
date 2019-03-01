module BenchmarkInputDataGen

export write_csv_files

using RigidBodyDynamics
using DelimitedFiles
using ForwardDiff
using RigidBodyDynamics: OneDegreeOfFreedomFixedAxis

using InteractiveUtils: subtypes
using Base.Iterators: flatten
using StaticArrays: SVector

import Random

const JointSegmentedVector{T} = RigidBodyDynamics.CustomCollections.SegmentedVector{JointID, T, Base.OneTo{JointID}, Vector{T}}

include("sample.jl")
include("csvwriters.jl")

function create_mechanism(urdf::String; floating::Bool)
    root_joint_type = floating ? QuaternionFloating{Float64}() : Fixed{Float64}()
    mechanism = parse_urdf(urdf, root_joint_type=root_joint_type)
    mechanism.gravitational_acceleration = zero(mechanism.gravitational_acceleration)
    mechanism
end

"""
    write_csv_files(urdf; num_samples, csvdir, floating=false)

Write CSV file(s) containing randomly generated benchmark data for
a mechanism parsed from URDF `urdf`.

Keyword arguments:

* `num_samples`: number of samples to generate
* `csvdir`: directory in which to store CSV files
* `floating`: whether or not to insert a floating joint.

The output CSV files have a header with column names. The CSV contains
(respectively):

1. Joint positions ('q_JOINTNAME_SUFFIX')
2. Joint velocities ('v_JOINTNAME_SUFFIX')
3. Joint accelerations ('vd_JOINTNAME_SUFFIX')
4. Joint torques ('tau_JOINTNAME_SUFFIX')
"""
function write_csv_files(urdf; num_samples, csvdir, floating=false)
    mechanism = create_mechanism(urdf, floating=floating)

    state = MechanismState(mechanism)
    result = DynamicsResult(mechanism)
    τ = fill!(similar(velocity(state)), NaN)
    samples = Sample[]
    sizehint!(samples, num_samples)

    floating_joint_velocity_range = if floating
        floating_joint = first(joints(mechanism))
        velocity_range(state, floating_joint)
    else
        1:0
    end

    Random.seed!(1)
    for _ in 1 : num_samples
        Random.rand!(state)
        Random.rand!(τ)
        τ[floating_joint_velocity_range] .= 0
        dynamics!(result, state, τ)
        sample = Sample(copy(configuration(state)), copy(velocity(state)), copy(result.v̇), copy(τ))
        push!(samples, sample)
    end

    filename = basename(urdf)
    mkpath(csvdir)
    for Writer in subtypes(CSVWriter)
        robotname, _ = splitext(filename)
        writer = Writer(mechanism, robotname)
        libname = shortlibname(writer)
        libdir = joinpath(csvdir, libname)
        mkpath(libdir)
        write_inputs_csv(writer, libdir, robotname, samples)
        write_expected_result_csvs(writer, libdir, robotname, first(samples))
    end
end

struct URDFConfig
    basename::String
    floating::Bool
end

const URDF_DIR = joinpath(@__DIR__, "..", "..", "description", "urdf")
const URDF_CONFIGS = [URDFConfig("iiwa.urdf", false), URDFConfig("atlas.urdf", true), URDFConfig("hyq.urdf", true)]
const CSV_DIR = joinpath(@__DIR__, "..", "..", "csv")

function write_csv_files(;urdfdir=URDF_DIR, csvdir=CSV_DIR, num_samples=2)
    for config in URDF_CONFIGS
        write_csv_files(joinpath(urdfdir, config.basename); num_samples=num_samples, csvdir=csvdir, floating=config.floating)
    end
end

end # module
