abstract type CSVWriter end

write_csv_row!(io::IO, vec::AbstractVector) = writedlm(io, reshape(vec, 1, :), ",")
write_csv_row!(io::IO, array::AbstractArray) = write_csv_row!(io, vec(array))

function write_inputs_csv(writer::CSVWriter, dir::String, robot_name::String, samples::AbstractVector{<:Sample})
    open(joinpath(dir, "$(robot_name)_inputs.csv"), "w") do io
        header = make_header(writer)
        write_csv_row!(io, header)
        for sample in samples
            sampledata = make_sample_data(writer, sample)
            write_csv_row!(io, vcat(sampledata...))
        end
    end
end

function write_expected_result_csvs(writer::CSVWriter, dir::String, robot_name::String, sample::Sample)
    Jlib = velocity_jacobian(writer, sample)

    state = MechanismState(writer.mechanism)
    set_configuration!(state, sample.q)
    set_velocity!(state, sample.v)
    M = mass_matrix(state)
    τid = inverse_dynamics(state, sample.v̇)
    c = dynamics_bias(state)

    Mlib = (Jlib' \ M) * Jlib'
    # τidlib = Jlib' \ τid
    # τlib = Jlib' \ sample.τ
    # clib = Jlib' \ c
    _, _, v̇lib, τlib = make_sample_data(writer, sample)

    open(joinpath(dir, "$(robot_name)_inverse_dynamics_expected.csv"), "w") do io
        write_csv_row!(io, τlib)
    end

    open(joinpath(dir, "$(robot_name)_mass_matrix_expected.csv"), "w") do io
        write_csv_row!(io, Mlib)
    end

    open(joinpath(dir, "$(robot_name)_dynamics_expected.csv"), "w") do io
        write_csv_row!(io, v̇lib)
    end
end

function make_header(writer::CSVWriter)
    header = String[]
    append!(header, configuration_coordinate_names(writer))
    append!(header, tangential_coordinate_names(writer, "v"))
    append!(header, tangential_coordinate_names(writer, "vd"))
    append!(header, tangential_coordinate_names(writer, "tau"))
    header
end

function configuration_coordinate_names(joint::Joint{<:Any, <:OneDegreeOfFreedomFixedAxis}, ::CSVWriter)
    ["q_$(string(joint))"]
end

function tangential_coordinate_names(joint::Joint{<:Any, <:OneDegreeOfFreedomFixedAxis}, prefix::String, ::CSVWriter)
    ["$(prefix)_$(string(joint))"]
end

function velocity_jacobian(writer::CSVWriter, sample::Sample)
    ForwardDiff.jacobian(sample.v) do v
        T = eltype(v)
        sample_dual = Sample(copyto!(similar(sample.q, T), sample.q), v, copyto!(similar(sample.v̇, T), sample.v̇), copyto!(similar(sample.τ, T), sample.τ))
        _, v_lib, _, _ = make_sample_data(writer, sample_dual)
        v_lib
    end
end


# RigidBodyDynamics.jl
struct RigidBodyDynamicsJlWriter <:CSVWriter
    mechanism::Mechanism{Float64}
end

RigidBodyDynamicsJlWriter(mechanism::Mechanism, robot_name::String) = RigidBodyDynamicsJlWriter(mechanism)

shortlibname(::RigidBodyDynamicsJlWriter) = "rbdjl"

function configuration_coordinate_names(writer::RigidBodyDynamicsJlWriter)
    configuration_coordinate_names.(tree_joints(writer.mechanism), Ref(writer)) |> flatten |> collect
end

function tangential_coordinate_names(writer::RigidBodyDynamicsJlWriter, prefix::String)
    tangential_coordinate_names.(tree_joints(writer.mechanism), Ref(prefix), Ref(writer)) |> flatten |> collect
end

function make_sample_data(writer::RigidBodyDynamicsJlWriter, sample::Sample)
    (sample.q, sample.v, sample.v̇, sample.τ)
end

function configuration_coordinate_names(joint::Joint{<:Any, <:QuaternionFloating}, ::RigidBodyDynamicsJlWriter)
    jointprefix = "q_$(string(joint))"
    [
        jointprefix * "_qs",
        jointprefix * "_qx",
        jointprefix * "_qy",
        jointprefix * "_qz",
        jointprefix * "_x",
        jointprefix * "_y",
        jointprefix * "_z",
    ]
end

function tangential_coordinate_names(joint::Joint{<:Any, <:QuaternionFloating}, prefix::String, ::RigidBodyDynamicsJlWriter)
    jointprefix = "$(prefix)_$(string(joint))"
    [
        jointprefix * "_wx",
        jointprefix * "_wy",
        jointprefix * "_wz",
        jointprefix * "_x",
        jointprefix * "_y",
        jointprefix * "_z",
    ]
end


# RBDL
struct RBDLWriter{S<:StateCache} <: CSVWriter
    mechanism::Mechanism
    statecache::S
    joints::Vector{Joint{Float64}}
    q_permutation::Vector{Int}
end

const RBDL_JOINT_NAMES = Dict{String, Vector{String}}()
RBDL_JOINT_NAMES["iiwa"] = [
    "lbr_iiwa_joint_1",
    "lbr_iiwa_joint_2",
    "lbr_iiwa_joint_3",
    "lbr_iiwa_joint_4",
    "lbr_iiwa_joint_5",
    "lbr_iiwa_joint_6",
    "lbr_iiwa_joint_7"
]
RBDL_JOINT_NAMES["hyq"] = [
    "base_link_0_to_world",
    "lf_haa_joint",
    "lf_hfe_joint",
    "lf_kfe_joint",
    "lh_haa_joint",
    "lh_hfe_joint",
    "lh_kfe_joint",
    "rf_haa_joint",
    "rf_hfe_joint",
    "rf_kfe_joint",
    "rh_haa_joint",
    "rh_hfe_joint",
    "rh_kfe_joint"
]
RBDL_JOINT_NAMES["atlas"] = [
    "pelvis_to_world",
    "back_bkz",
    "back_bky",
    "back_bkx",
    "l_arm_shz",
    "l_arm_shx",
    "l_arm_ely",
    "l_arm_elx",
    "l_arm_uwy",
    "l_arm_mwx",
    "l_arm_lwy",
    "neck_ay",
    "r_arm_shz",
    "r_arm_shx",
    "r_arm_ely",
    "r_arm_elx",
    "r_arm_uwy",
    "r_arm_mwx",
    "r_arm_lwy",
    "l_leg_hpz",
    "l_leg_hpx",
    "l_leg_hpy",
    "l_leg_kny",
    "l_leg_aky",
    "l_leg_akx",
    "r_leg_hpz",
    "r_leg_hpx",
    "r_leg_hpy",
    "r_leg_kny",
    "r_leg_aky",
    "r_leg_akx"
]

function RBDLWriter(mechanism::Mechanism, robot_name::String)
    # q order is permuted so that quaternion scalar components come last.
    # https://bitbucket.org/rbdl/rbdl/src/0879ee8c548a07b8a9e1296bcbb5b006e1a098c7/src/Model.cc#lines-374:375
    joint_names = RBDL_JOINT_NAMES[robot_name]
    joints_rbdl = Vector{Joint{Float64}}(undef, length(joints(mechanism)))
    for i in eachindex(joints_rbdl)
        joints_rbdl[i] = findjoint(mechanism, joint_names[i])
    end
    @assert isempty(setdiff(joints_rbdl, tree_joints(mechanism)))
    state = MechanismState(mechanism)
    q_permutation = Int[]
    quatjoints = Joint[]
    for joint in joints_rbdl
        qrange = configuration_range(state, joint)
        vrange = velocity_range(state, joint)
        if joint_type(joint) isa OneDegreeOfFreedomFixedAxis
            append!(q_permutation, qrange)
        elseif joint_type(joint) isa QuaternionFloating
            push!(quatjoints, joint)
            append!(q_permutation, qrange[5 : 7])
            append!(q_permutation, qrange[2 : 4])
        else
            error("Joint type not supported")
        end
    end
    for joint in quatjoints
        push!(q_permutation, configuration_range(state, joint)[1])
    end
    RBDLWriter(mechanism, StateCache(mechanism), joints_rbdl, q_permutation)
end

shortlibname(::RBDLWriter) = "rbdl"

function configuration_coordinate_names(joint::Joint{<:Any, <:QuaternionFloating}, ::RBDLWriter)
    # NOTE: still transformed by q_permutation after (easier that way)
    jointprefix = "q_$(string(joint))"
    [
        jointprefix * "_qs",
        jointprefix * "_qx",
        jointprefix * "_qy",
        jointprefix * "_qz",
        jointprefix * "_x",
        jointprefix * "_y",
        jointprefix * "_z",
    ]
end

function tangential_coordinate_names(joint::Joint{<:Any, <:QuaternionFloating}, prefix::String, ::RBDLWriter)
    jointprefix = "$(prefix)_$(string(joint))"
    [
        jointprefix * "_x",
        jointprefix * "_y",
        jointprefix * "_z",
        jointprefix * "_wx",
        jointprefix * "_wy",
        jointprefix * "_wz"
    ]
end

function configuration_coordinate_names(writer::RBDLWriter)
    names = configuration_coordinate_names.(tree_joints(writer.mechanism), Ref(writer)) |> flatten |> collect
    names[writer.q_permutation]
end

function tangential_coordinate_names(writer::RBDLWriter, prefix::String)
    tangential_coordinate_names.(writer.joints, Ref(prefix), Ref(writer)) |> flatten |> collect
end

function make_sample_data(writer::RBDLWriter, sample::Sample{T}) where T
    mechanism = writer.mechanism
    state = writer.statecache[T]
    rootframe = root_frame(mechanism)
    set_configuration!(state, sample.q)
    set_velocity!(state, sample.v)

    # q
    q_rbdl = sample.q[writer.q_permutation]

    # v, v̇, τ
    v_rbdl = fill!(similar(velocity(state)), NaN)
    v̇_rbdl = fill!(similar(velocity(state)), NaN)
    τ_rbdl = fill!(similar(velocity(state)), NaN)
    vstart = 1
    for joint in writer.joints
        jointid = JointID(joint)
        vrange = velocity_range(state, jointid)
        if joint_type(joint) isa OneDegreeOfFreedomFixedAxis
            v_rbdl[vstart : vstart] .= sample.v[vrange]
            v̇_rbdl[vstart : vstart] .= sample.v̇[vrange]
            τ_rbdl[vstart : vstart] .= sample.τ[vrange]
        elseif joint_type(joint) isa QuaternionFloating
            # RBDL rewrites
            #   [world] -- floating joint -- [floating body]
            # as
            #   [world] -- translation xyz -- [null body] -- spherical -- [floating body]
            # The velocity for an RBDL translation xyz joint is just the time derivative of the position.
            # The velocity for an RBDL spherical joint is the angular velocity in body frame
            # (https://bitbucket.org/rbdl/rbdl/src/0879ee8c548a07b8a9e1296bcbb5b006e1a098c7/include/rbdl/Joint.h#lines-135:144).

            v̇joint = sample.v̇[jointid]
            τjoint = sample.τ[jointid]
            jointtf = joint_transform(state, joint)
            jointtwist = RigidBodyDynamics.twist(state, joint)
            jointaccel = SpatialAcceleration(jointtwist.body, jointtwist.base, jointtwist.frame, SVector{3}(v̇joint[1 : 3]), SVector{3}(v̇joint[4 : 6]))
            jointwrench = Wrench(frame_after(joint), SVector{3}(τjoint[1 : 3]), SVector{3}(τjoint[4 : 6]))

            body_origin = Point3D(jointtwist.body, zero(T), zero(T), zero(T))
            body_origin_velocity = transform(point_velocity(jointtwist, body_origin), jointtf)
            body_origin_accel = transform(point_acceleration(jointtwist, jointaccel, body_origin), jointtf)

            # v
            v_rbdl[vstart .+ (0 : 2)] .= body_origin_velocity.v
            v_rbdl[vstart .+ (3 : 5)] .= angular(jointtwist)

            # v̇
            v̇_rbdl[vstart .+ (0 : 2)] .= body_origin_accel.v
            v̇_rbdl[vstart .+ (3 : 5)] .= angular(jointaccel)

            # τ
            τ_rbdl[vstart .+ (0 : 2)] .= linear(transform(jointwrench, jointtf))
            τ_rbdl[vstart .+ (3 : 5)] .= angular(jointwrench)
        else
            error("Joint type not yet supported")
        end
        vstart += num_velocities(joint)
    end

    (q_rbdl, v_rbdl, v̇_rbdl, τ_rbdl)
end


# Pinocchio
struct PinocchioWriter <: CSVWriter
    mechanism::Mechanism{Float64}
    q_permutation::Vector{Int}
    v_permutation::Vector{Int}
end

const PINOCCHIO_JOINT_NAMES = Dict{String, Vector{String}}()
PINOCCHIO_JOINT_NAMES["iiwa"] = [
    "lbr_iiwa_joint_1",
    "lbr_iiwa_joint_2",
    "lbr_iiwa_joint_3",
    "lbr_iiwa_joint_4",
    "lbr_iiwa_joint_5",
    "lbr_iiwa_joint_6",
    "lbr_iiwa_joint_7"
]
PINOCCHIO_JOINT_NAMES["hyq"] = [
    "base_link_0_to_world",
    "lf_haa_joint",
    "lf_hfe_joint",
    "lf_kfe_joint",
    "lh_haa_joint",
    "lh_hfe_joint",
    "lh_kfe_joint",
    "rf_haa_joint",
    "rf_hfe_joint",
    "rf_kfe_joint",
    "rh_haa_joint",
    "rh_hfe_joint",
    "rh_kfe_joint"
]
PINOCCHIO_JOINT_NAMES["atlas"] = [
    "pelvis_to_world",
    "back_bkz",
    "back_bky",
    "back_bkx",
    "l_arm_shz",
    "l_arm_shx",
    "l_arm_ely",
    "l_arm_elx",
    "l_arm_uwy",
    "l_arm_mwx",
    "l_arm_lwy",
    "neck_ay",
    "r_arm_shz",
    "r_arm_shx",
    "r_arm_ely",
    "r_arm_elx",
    "r_arm_uwy",
    "r_arm_mwx",
    "r_arm_lwy",
    "l_leg_hpz",
    "l_leg_hpx",
    "l_leg_hpy",
    "l_leg_kny",
    "l_leg_aky",
    "l_leg_akx",
    "r_leg_hpz",
    "r_leg_hpx",
    "r_leg_hpy",
    "r_leg_kny",
    "r_leg_aky",
    "r_leg_akx"
]

function PinocchioWriter(mechanism::Mechanism, robot_name::String)
    joint_names = PINOCCHIO_JOINT_NAMES[robot_name]
    joints_pinocchio = findjoint.(Ref(mechanism), joint_names)
    @assert isempty(setdiff(joints_pinocchio, tree_joints(mechanism)))
    state = MechanismState(mechanism)
    q_permutation = Int[]
    v_permutation = Int[]
    for joint in joints_pinocchio
        qrange = configuration_range(state, joint)
        vrange = velocity_range(state, joint)
        if joint_type(joint) isa OneDegreeOfFreedomFixedAxis
            append!(q_permutation, qrange)
            append!(v_permutation, vrange)
        elseif joint_type(joint) isa QuaternionFloating
            append!(q_permutation, qrange[[5, 6, 7, 2, 3, 4, 1]])
            append!(v_permutation, vrange[[4, 5, 6, 1, 2, 3]])
        else
            error("Joint type not supported")
        end
    end
    @assert length(q_permutation) == num_positions(mechanism)
    @assert length(v_permutation) == num_velocities(mechanism)
    PinocchioWriter(mechanism, q_permutation, v_permutation)
end

shortlibname(::PinocchioWriter) = "pinocchio"

function configuration_coordinate_names(writer::PinocchioWriter)
    names = configuration_coordinate_names.(tree_joints(writer.mechanism), Ref(writer)) |> flatten |> collect
    names[writer.q_permutation]
end

function tangential_coordinate_names(writer::PinocchioWriter, prefix::String)
    names = tangential_coordinate_names.(tree_joints(writer.mechanism), Ref(prefix), Ref(writer)) |> flatten |> collect
    names[writer.v_permutation]
end

function make_sample_data(writer::PinocchioWriter, sample::Sample)
    q_pinocchio = sample.q[writer.q_permutation]
    v_pinocchio = sample.v[writer.v_permutation]
    v̇_pinocchio = sample.v̇[writer.v_permutation]
    τ_pinocchio = sample.τ[writer.v_permutation]
    (q_pinocchio, v_pinocchio, v̇_pinocchio, τ_pinocchio)
end

function configuration_coordinate_names(joint::Joint{<:Any, <:QuaternionFloating}, ::PinocchioWriter)
    # NOTE: still transformed by q_permutation after (easier that way)
    jointprefix = "q_$(string(joint))"
    [
        jointprefix * "_qs",
        jointprefix * "_qx",
        jointprefix * "_qy",
        jointprefix * "_qz",
        jointprefix * "_x",
        jointprefix * "_y",
        jointprefix * "_z",
    ]
end

function tangential_coordinate_names(joint::Joint{<:Any, <:QuaternionFloating}, prefix::String, ::PinocchioWriter)
    # NOTE: still transformed by v_permutation after (easier that way)
    jointprefix = "$(prefix)_$(string(joint))"
    [
        jointprefix * "_wx",
        jointprefix * "_wy",
        jointprefix * "_wz",
        jointprefix * "_x",
        jointprefix * "_y",
        jointprefix * "_z",
    ]
end


# RobCoGen
struct RobCoGenWriter <:CSVWriter
    mechanism::Mechanism{Float64}
    non_floating_joints::Vector{Joint{Float64}}
    non_floating_q_indices::Vector{Int}

    function RobCoGenWriter(mechanism::Mechanism{Float64}, robot_name::String)
        non_floating_joints = Joint{Float64}[j for j in tree_joints(mechanism) if !isfloating(j)]
        state = MechanismState(mechanism)
        non_floating_q_indices = [configuration_range(state, j) for j in non_floating_joints] |> flatten |> collect
        new(mechanism, non_floating_joints, non_floating_q_indices)
    end
end

shortlibname(::RobCoGenWriter) = "robcogen"

function tangential_coordinate_names(joint::Joint{<:Any, <:QuaternionFloating}, prefix::String, ::RobCoGenWriter)
    jointprefix = "$(prefix)_$(string(joint))"
    [
        jointprefix * "_wx",
        jointprefix * "_wy",
        jointprefix * "_wz",
        jointprefix * "_x",
        jointprefix * "_y",
        jointprefix * "_z",
    ]
end

function configuration_coordinate_names(writer::RobCoGenWriter)
    configuration_coordinate_names.(writer.non_floating_joints, Ref(writer)) |> flatten |> collect
end

function tangential_coordinate_names(writer::RobCoGenWriter, prefix::String)
    tangential_coordinate_names.(tree_joints(writer.mechanism), Ref(prefix), Ref(writer)) |> flatten |> collect
end

function make_sample_data(writer::RobCoGenWriter, sample::Sample)
    (sample.q[writer.non_floating_q_indices], sample.v, sample.v̇, sample.τ)
end
