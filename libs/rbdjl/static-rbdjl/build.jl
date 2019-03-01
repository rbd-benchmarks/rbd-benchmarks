using PackageCompiler

builddir = ARGS[1]
mkpath(builddir)

build_shared_lib(
    joinpath(@__DIR__, "staticrbd.jl"), "libstaticrbdjl",
    snoopfile = joinpath(@__DIR__, "snoopfile.jl"),
    startup_file = "no",
    builddir = builddir,
    Release = true,
    check_bounds = "no",
    optimize = 3
)
