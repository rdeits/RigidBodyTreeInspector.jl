using Base.Test
using RigidBodyTreeInspector
using RigidBodyDynamics

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
    for link in vis.robot.links
        @test !(link.name in previous_names)
        push!(previous_names, link.name)
    end
end

@testset "urdf mechanism" begin
    urdf = "$(ENV["HOME"])/locomotion/drake-distro/drake/examples/Valkyrie/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"
    mechanism = RigidBodyDynamics.parse_urdf(Float64, urdf)
    package_path = ["$(ENV["HOME"])/locomotion/drake-distro/drake/examples"]
    vis = RigidBodyTreeInspector.parse_urdf(urdf, mechanism; package_path=package_path)
    @test length(vis.robot.links) == length(bodies(mechanism))
end

@testset "notebooks" begin
    if VERSION < v"0.6-dev"  # skip notebooks on 0.6 due to jupyter kernel version mismatch
        using IJulia
        jupyter = IJulia.jupyter

        for notebook in ["../examples/demo.ipynb", "../examples/urdf.ipynb"]
            run(`$jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
        end
    end
end
