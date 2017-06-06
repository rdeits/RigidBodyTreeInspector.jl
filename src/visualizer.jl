to_link_name(frame::CartesianFrame3D) =
    Symbol("$(RigidBodyDynamics.name(frame))_(#$(frame.id))")


function setgeometry!(vis::Visualizer,
                      frame_geometries::Associative{CartesianFrame3D, Vector{GeometryData}})
    batch(vis) do v
        delete!(v)
        for (frame, geoms) in frame_geometries
            for (i, geom) in enumerate(geoms)
                setgeometry!(v[to_link_name(frame)][Symbol("geometry$i")], geom)
            end
        end
    end
end

function settransform!(vis::Visualizer, state::MechanismState)
    batch(vis) do v
        for body in bodies(state.mechanism)
            for transform in RigidBodyDynamics.frame_definitions(body)
                frame = transform.from
                framename = to_link_name(frame)
                if framename in keys(vis.core.tree[vis.path].children)
                    framevis = v[to_link_name(frame)]
                    settransform!(framevis,
                                  convert(AffineMap,
                                          transform_to_root(state, frame)))
                end
            end
        end
    end
end

"""
Construct a DrakeVisualizer.Visualizer for the given mechanism by constructing
simple geometries just from the structure of the kinematic tree. If
`show_intertias` is true, then also construct equivalent inertial ellipsoids
for every link.
"""
function Visualizer(mechanism::Mechanism, prefix=[:robot1];
                    show_inertias::Bool=false, randomize_colors::Bool=true)
    frame_geoms = create_geometry(mechanism; show_inertias=show_inertias, randomize_colors=randomize_colors)
    Visualizer(frame_geoms, prefix)
end

function Visualizer(frame_geometries::Associative{CartesianFrame3D, Vector{GeometryData}},
                    prefix=[:robot1])
    vis = Visualizer()[prefix]
    setgeometry!(vis, frame_geometries)
    vis
end

convert(::Type{AffineMap}, T::Transform3D) =
    AffineMap(rotation(T), translation(T))

inspect!(state::MechanismState, vis::Visualizer) = manipulate!(state) do state
    settransform!(vis, state)
end

function inspect!(state::MechanismState;
        show_inertias::Bool=false, randomize_colors::Bool=true)
    vis = Visualizer()[:robot1]
    setgeometry!(vis, create_geometry(state.mechanism;
                               show_inertias=show_inertias,
                               randomize_colors=randomize_colors))
    inspect!(state, vis)
end

inspect(mechanism::Mechanism, args...; kwargs...) =
    inspect!(MechanismState(Float64, mechanism), args...; kwargs...)
