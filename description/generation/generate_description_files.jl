using RigidBodyDynamics

using RigidBodyDynamics: Bounds

const urdfdir = joinpath(@__DIR__, "..", "urdf")
const kindsldir = joinpath(@__DIR__, "..", "kindsl")
const sourcedir = joinpath(urdfdir, "source")
const simplifieddir = joinpath(urdfdir, "simplified")

const urdf2kindsl = joinpath(@__DIR__, "..", "urdf2kindsl", "urdf2kindsl.py")
const libdir = joinpath(@__DIR__, "..", "..", "libs")
const robcogen = joinpath(libdir, "robcogen", "robcogen", "exe", "robcogen.sh")
const robcogen_gen_dir = "/tmp/gen"

for urdf in filter(x -> endswith(x, ".urdf"), readdir(sourcedir))
    println("Processing $urdf")
    robot_name, _ = splitext(urdf)
    floatingfile = joinpath(sourcedir, "$(robot_name)_floating")
    isfile(floatingfile) || error("$floatingfile not found")
    floating = open(floatingfile) do io
        Bool(parse(Int, read(io, String)))
    end

    # Generate simplified URDF from raw URDF (remove fixed joints, change root joint type,
    # work around RBDL bug where links named "base_link" are misparsed)
    root_joint_type = floating ? QuaternionFloating{Float64}() : Fixed{Float64}()
    mechanism = parse_urdf(joinpath(sourcedir, urdf), root_joint_type=root_joint_type)
    for body in bodies(mechanism)
        if body.name == "base_link"
            i = 0
            while true
                new_name = "base_link_$i"
                if new_name ∈ map(b -> b.name, bodies(mechanism))
                    i += 1
                else
                    body.name = new_name
                    break
                end
            end
        end
    end
    simplified_urdf = joinpath(simplifieddir, "$robot_name.urdf")
    write_urdf(simplified_urdf, mechanism; robot_name=robot_name, include_root=!floating)

    # Generate kindsl files that are at least approximately the same using `urdf2kindsl`.
    # We suspect that that urdf2kindsl has a flaw that makes it produce .kindsl files that
    # aren't quite equivalent to the URDF files in some cases (notably, Atlas).
    kindsl = joinpath(kindsldir, "$robot_name.kindsl")
    run(pipeline(`$urdf2kindsl --digits=12 --pi-digits=12 $simplified_urdf`, stdout=kindsl))

    # Change `RobotBase x` to `RobotBase x floating` as appropriate
    if floating
        modified_kindsl_contents = replace(read(kindsl, String), r"(RobotBase\s*\S*)" => s"\g<0> floating")
        write(kindsl, modified_kindsl_contents)
    end

    # Generate URDFs from kindsl files
    robcogen_gen_urdf_option = 23
    robcogen_quit_option = 28
    cd(dirname(robcogen)) do # robcogen.sh needs to be run from its parent directory
        open(pipeline(`$robcogen $kindsl`, stdout=devnull), "w", stdout) do io
            println(io, robcogen_gen_urdf_option)
            println(io, robcogen_quit_option)
        end
    end
    robcogen_urdf = joinpath(robcogen_gen_dir, robot_name, "models", urdf)

    # Remove explicit floating joints
    mechanism = parse_urdf(robcogen_urdf, root_joint_type=Fixed{Float64}())
    root = root_body(mechanism)
    floating_joints = filter(isfloating, out_joints(root, mechanism))
    convert(Bool, length(floating_joints)) == floating || error()
    if !isempty(floating_joints)
        mechanism, = submechanism(mechanism, successor(floating_joints[1], mechanism))
    end

    # RBDL doesn't handle missing inertia tag for root body properly, so add one explicitly:
    root = root_body(mechanism)
    if root.inertia == nothing
        root.inertia = zero(SpatialInertia{Float64}, root_frame(mechanism))
    end

    # Pinocchio seems to misparse the 'continuous' joint type (model.nq is double what it should be)
    # so add some artificial joint limits, causing RigidBodyDynamics.jl to output 'revolute' rather
    # than 'continuous' joint types
    for joint in joints(mechanism)
        if joint_type(joint) isa Revolute
            joint.position_bounds .= Bounds(-1e5, 1e5)
        end
    end

    # Write final result to `urdfdir`
    write_urdf(joinpath(urdfdir, urdf), mechanism; robot_name=robot_name, include_root=true)
end
