"""
Construct a DrakeVisualizer.Visualizer for the given mechanism by constructing
simple geometries just from the structure of the kinematic tree. If
`show_intertias` is true, then also construct equivalent inertial ellipsoids
for every link.
"""
function Visualizer(mechanism::Mechanism;
                    show_inertias::Bool=false, randomize_colors::Bool=true)
    vis_data = create_geometry(mechanism;
                               show_inertias=show_inertias,
                               randomize_colors=randomize_colors)
    Visualizer(collect(values(vis_data)))
end

convert(::Type{AffineMap}, T::Transform3D) =
    AffineMap(RigidBodyDynamics.rotationmatrix_normalized_fsa(T.rot), T.trans)

function draw(vis::Visualizer, state::MechanismState)
    bodies = [vertex_data(v) for v in state.mechanism.toposortedTree]
    origin_transforms = map(body -> convert(AffineMap, transform_to_root(state, RigidBodyDynamics.default_frame(state.mechanism, body))), bodies)
    draw(vis, origin_transforms)
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
