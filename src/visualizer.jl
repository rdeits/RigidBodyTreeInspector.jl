unique_frame_name(frame::CartesianFrame3D) =
    Symbol("$(frame)_(#$(frame.id))")

Base.@deprecate to_link_name(frame::CartesianFrame3D) unique_frame_name(frame)

const FrameGeometries = Associative{CartesianFrame3D, <:AbstractVector}

function setgeometry!(vis::Visualizer, mechanism::Mechanism, frame_geometries::FrameGeometries=create_geometry(mechanism))
    batch(vis) do v
        delete!(v)
        addgeometry!(vis, mechanism, frame_geometries)
    end
    nothing
end

function addgeometry!(vis::Visualizer, mechanism::Mechanism, frame_geometries::FrameGeometries=create_geometry(mechanism))
    body_names = Set{Symbol}()
    batch(vis) do v
        for body in bodies(mechanism)
            body_name = Symbol(body)
            if body_name in body_names
                error("Duplicate body name in mechanism")
            end
            push!(body_names, body_name)
            bodyvis = v[body_name]
            found_geometry = false
            for transform in rbd.frame_definitions(body)
                frame = transform.from
                framename = unique_frame_name(frame)
                if haskey(frame_geometries, frame)
                    found_geometry = true
                    for geometry in frame_geometries[frame]
                        addgeometry!(bodyvis[framename], geometry)
                    end
                    settransform!(bodyvis[framename], rbd.frame_definition(body, frame))
                end
            end
            if !found_geometry
                setgeometry!(bodyvis, [])
            end
        end
    end
    nothing
end

function addgeometry!(vis::Visualizer, mechanism::Mechanism, frame::CartesianFrame3D, geometry)
    addgeometry!(vis, mechanism, Dict(frame => [geometry]))
end

function addgeometry!(vis::Visualizer, mechanism::Mechanism, frame::CartesianFrame3D; scale=1.0)
    addgeometry!(vis, mechanism, frame, Triad(scale, false))
end

function addgeometry!(vis::Visualizer, mechanism::Mechanism, point::Point3D; radius=0.03)
    addgeometry!(vis, mechanism, point.frame, HyperSphere(Point{3, Float64}(point.v), radius))
end

function setgeometry!(vis::Visualizer,
                      frame_geometries::Associative{CartesianFrame3D, Vector{GeometryData}})
    error("""
setgeometry!(vis, frame_geometries) has been updated to take the Mechanism
object as its second argument. Please call it using the syntax:

    setgeometry!(vis, mechanism, frame_geometries)

For example, if you were previously doing

    setgeometry!(vis, create_geometry(mechanism))

you should now call:

    setgeometry!(vis, mechanism, create_geometry(mechanism))

which is equivalent to the new, simpler version:

    setgeometry!(vis, mechanism)
""")
end

settransform!(vis::Visualizer, tform::rbd.Transform3D) = settransform!(vis, convert(AffineMap, tform))

function settransform!(vis::Visualizer, state::MechanismState)
    batch(vis) do v
        for body in bodies(state.mechanism)
            body_name = Symbol(body)
            if body_name in keys(v.core.tree[v.path].children)
                settransform!(v[body_name], transform_to_root(state, body))
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
    vis = Visualizer()[prefix]
    setgeometry!(vis, mechanism, frame_geoms)
    vis
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
    setgeometry!(vis, state.mechanism,
                 create_geometry(state.mechanism;
                                 show_inertias=show_inertias,
                                 randomize_colors=randomize_colors))
    inspect!(state, vis)
end

inspect(mechanism::Mechanism, args...; kwargs...) =
    inspect!(MechanismState{Float64}(mechanism), args...; kwargs...)
