using Base.Test
using RigidBodyTreeInspector
using RigidBodyDynamics
using IJulia

@testset "chain mechanism" begin
    mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
    vis = Visualizer(mechanism; show_inertias=true);
    # We can draw the mechanism at a single state:
    state = MechanismState(Float64, mechanism)
    rand!(state)
    draw(vis, state)
end

@testset "attach mechanism" begin
    mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
    mechanism2 = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1:5]]...)
    attach!(mechanism, root_body(mechanism), mechanism2)
    vis = Visualizer(mechanism)
    previous_names = Set()
    for frame in keys(vis.links)
        name = DrakeVisualizer.to_link_name(frame)
        @test !(name in previous_names)
        push!(previous_names, name)
    end
end

@testset "simulation and animation" begin
    mechanism = rand_chain_mechanism(Float64, [Revolute{Float64} for i = 1:10]...)
    vis = Visualizer(mechanism, show_inertias=true);
    state = MechanismState(Float64, mechanism)
    zero!(state)
    draw(vis, state)
    times, states = simulate(state, 1);
    animate(vis, mechanism, times, states)
end

@testset "urdf mechanism" begin
    urdf = joinpath(dirname(@__FILE__), "..", "examples", "urdf", "Acrobot.urdf")
    mechanism = RigidBodyDynamics.parse_urdf(Float64, urdf)
    vis = RigidBodyTreeInspector.parse_urdf(urdf, mechanism)
end

test_notebook(notebook) =
    run(`$(IJulia.jupyter) nbconvert --to notebook --execute $notebook --output $notebook`)

@testset "notebooks" begin
    test_notebook("../examples/demo.ipynb")

    if isfile(get(ENV, "DRAKE_DISTRO", "")) || isfile(joinpath(ENV["HOME"], "locomotion", "drake-distro"))
        # Only run this test on my machine or on my Travis build
        test_notebook("../examples/urdf.ipynb")
    end
end
