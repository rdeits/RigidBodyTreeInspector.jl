to_link_name(frame::CartesianFrame3D) =
    "$(RigidBodyDynamics.name(frame))_(#$(frame.id))"

"""
Construct a DrakeVisualizer.Visualizer for the given mechanism by constructing
simple geometries just from the structure of the kinematic tree. If
`show_intertias` is true, then also construct equivalent inertial ellipsoids
for every link.
"""
function Visualizer(mechanism::Mechanism;
                    show_inertias::Bool=false, randomize_colors::Bool=true)
    Visualizer(create_geometry(mechanism;
                               show_inertias=show_inertias,
                               randomize_colors=randomize_colors))
end

convert(::Type{AffineMap}, T::Transform3D) =
    AffineMap(T.rot, T.trans)

function draw(vis::Visualizer, state::MechanismState)
    transforms = Dict(
        (frame, convert(AffineMap, transform_to_root(state, frame))) for frame in keys(vis.links))
    draw(vis, transforms)
end

inspect!(state::MechanismState, vis::Visualizer) = manipulate!(state) do state
    draw(vis, state)
end

inspect!(state::MechanismState;
        show_inertias::Bool=false, randomize_colors::Bool=true) =
    inspect!(state, Visualizer(state.mechanism;
                               show_inertias=show_inertias,
                               randomize_colors=randomize_colors))

inspect(mechanism::Mechanism, args...; kwargs...) =
    inspect!(MechanismState(Float64, mechanism), args...; kwargs...)
