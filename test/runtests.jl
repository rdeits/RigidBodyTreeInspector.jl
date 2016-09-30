using Base.Test
using RigidBodyTreeInspector

@testset "urdf mechanism" begin
    urdf = "$(ENV["HOME"])/locomotion/drake-distro/drake/examples/Valkyrie/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"
    mechanism = RigidBodyDynamics.parse_urdf(Float64, urdf)
    package_path = ["$(ENV["HOME"])/locomotion/drake-distro/drake/examples"]
    vis = RigidBodyTreeInspector.parse_urdf(urdf, mechanism; package_path=package_path)
end

@testset "notebooks" begin
    using IJulia
    jupyter = IJulia.jupyter

    for notebook in ["../examples/demo.ipynb", "../examples/urdf.ipynb"]
        run(`$jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
    end
end
