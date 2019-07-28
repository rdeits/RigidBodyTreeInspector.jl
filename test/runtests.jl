using Test
using RigidBodyTreeInspector
using RigidBodyDynamics
using DrakeVisualizer
using CoordinateTransformations
using ColorTypes
using NBInclude
using Random

@testset "chain mechanism" begin
    mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
    vis = Visualizer(mechanism; show_inertias=true);
    # We can draw the mechanism at a single state:
    state = MechanismState{Float64}(mechanism)
    rand!(state)
    settransform!(vis, state)
end

@testset "attach mechanism" begin
    mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
    urdf = joinpath(dirname(@__FILE__), "..", "examples", "urdf", "Acrobot.urdf")
    mechanism2 = parse_urdf(Float64, urdf)
    attach!(mechanism, root_body(mechanism), mechanism2)
    vis = Visualizer(mechanism)
end

@testset "simulation and animation" begin
    mechanism = rand_chain_mechanism(Float64, [Revolute{Float64} for i = 1:10]...)
    vis = Visualizer(mechanism, show_inertias=true);
    state = MechanismState{Float64}(mechanism)
    zero!(state)
    settransform!(vis, state)
    times, states = simulate(state, 1, Δt = 0.001);
    animate(vis, mechanism, times, states)
end

@testset "urdf mechanism" begin
    urdf = joinpath(dirname(@__FILE__), "..", "examples", "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)
    vis = Visualizer(mechanism, URDFVisuals(urdf))
    remove_fixed_tree_joints!(mechanism)
    vis = Visualizer(mechanism, URDFVisuals(urdf))
    mechanism = submechanism(mechanism, bodies(mechanism)[2])
    vis = Visualizer(mechanism, URDFVisuals(urdf))
end

@testset "geometry additions" begin
    urdf = joinpath(dirname(@__FILE__), "..", "examples", "urdf", "Acrobot.urdf")
    mechanism = parse_urdf(Float64, urdf)
    vis = Visualizer(mechanism, URDFVisuals(urdf))
    addgeometry!(vis, mechanism, default_frame(bodies(mechanism)[2]))
    addgeometry!(vis, mechanism, Point3D(default_frame(bodies(mechanism)[2]), 0.1, 0.1, 0.1))
    addgeometry!(vis, mechanism, default_frame(bodies(mechanism)[3]), HyperSphere(Point(0., 0, 0), 0.05))
    addgeometry!(vis, mechanism, default_frame(bodies(mechanism)[3]), GeometryData(HyperSphere(Point(0., 0, 0), 0.05), RGBA(0, 1, 0, 0.5), Translation(0.1, 0, 0)))
end

@testset "notebooks" begin
    @nbinclude(joinpath(@__DIR__, "..", "examples", "demo.ipynb"))
    @nbinclude(joinpath(@__DIR__, "..", "examples", "urdf.ipynb"))
end
