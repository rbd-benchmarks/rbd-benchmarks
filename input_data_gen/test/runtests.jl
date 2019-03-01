using Test
using BenchmarkInputDataGen
using DelimitedFiles
using LinearAlgebra
using RigidBodyDynamics
using ForwardDiff
using Random

using InteractiveUtils: subtypes
using Statistics: mean

function ranges(header::Vector{<:AbstractString})
    nq = count(x -> startswith(x, "q_"), header)
    nv = count(x -> startswith(x, "v_"), header)
    @assert length(header) == nq + 3 * nv
    start = 1
    qrange = start : (start += nq) - 1
    vrange = start : (start += nv) - 1
    v̇range = start : (start += nv) - 1
    τrange = start : (start += nv) - 1
    qrange, vrange, v̇range, τrange
end

@testset "write_csv_files" begin
    mktempdir() do csvdir
        num_samples = 3
        urdfdir = BenchmarkInputDataGen.URDF_DIR
        write_csv_files(urdfdir=urdfdir, csvdir=csvdir, num_samples=num_samples)

        for config in BenchmarkInputDataGen.URDF_CONFIGS
            mechanism = BenchmarkInputDataGen.create_mechanism(joinpath(urdfdir, config.basename), floating=config.floating)
            robotname, _ = splitext(config.basename)
            @test length(readdir(csvdir)) == length(subtypes(BenchmarkInputDataGen.CSVWriter))
            powers = Dict{String, Float64}()
            for libdir in readdir(csvdir)
                inputs_csv = joinpath(csvdir, libdir, "$(robotname)_inputs.csv")
                data, headerdata = open(inputs_csv, "r") do io
                    readdlm(io, ',', Float64, '\n'; header=true)
                end
                header = collect(String, vec(headerdata))

                @test size(data, 1) == num_samples
                @test length(header) > 0

                qrange, vrange, v̇range, τrange = ranges(header)

                @test all(x -> startswith(x, "q_"), header[qrange])
                @test all(x -> startswith(x, "v_"), header[vrange])
                @test all(x -> startswith(x, "vd_"), header[v̇range])
                @test all(x -> startswith(x, "tau_"), header[τrange])

                @test !any(isnan, data)
                for i = 1 : size(data, 1)
                    sampledata = data[i, :]
                    v = view(sampledata, vrange)
                    τ = view(sampledata, τrange)
                    powers[libdir] = v ⋅ τ
                end
            end
            @test all(x -> isapprox(x, mean(values(powers)); atol=1e-10), values(powers))
        end
    end
end


@testset "v / v̇: $(config.basename)" for config in BenchmarkInputDataGen.URDF_CONFIGS
    robotname, _ = splitext(config.basename)
    mechanism = BenchmarkInputDataGen.create_mechanism(joinpath(BenchmarkInputDataGen.URDF_DIR, config.basename), floating=config.floating)
    state = MechanismState(mechanism)
    Random.seed!(1)
    rand!(state)

    q = configuration(state)
    q̇ = configuration_derivative(state)
    v = velocity(state)
    v̇ = rand!(similar(velocity(state)))
    v̈ = zero(velocity(state))
    τ = rand!(similar(velocity(state)))
    τ̇ = zero(velocity(state))

    Tag = Nothing
    Dual = ForwardDiff.Dual{Tag, Float64, 1}
    q_dual = similar(configuration(state), Dual)
    v_dual = similar(velocity(state), Dual)
    v̇_dual = similar(velocity(state), Dual)
    τ_dual = similar(velocity(state), Dual)

    q_dual .= ForwardDiff.Dual.(q, q̇)
    v_dual .= ForwardDiff.Dual.(v, v̇)
    v̇_dual .= ForwardDiff.Dual.(v̇, v̈)
    τ_dual .= ForwardDiff.Dual.(τ, τ̇)
    sample = BenchmarkInputDataGen.Sample(q_dual, v_dual, v̇_dual, τ_dual)

    for Writer in subtypes(BenchmarkInputDataGen.CSVWriter)
        writer = Writer(mechanism, robotname)
        q_writer_dual, v_writer_dual, v̇_writer_dual, τ_writer_dual = BenchmarkInputDataGen.make_sample_data(writer, sample)
        v̇_writer_forwarddiff = ForwardDiff.extract_derivative(Tag, v_writer_dual)
        v̇_writer_direct = ForwardDiff.value.(v̇_writer_dual)
        @test v̇_writer_forwarddiff ≈ v̇_writer_direct
    end
end
