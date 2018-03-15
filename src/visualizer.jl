unique_frame_name(frame::CartesianFrame3D) =
    Symbol("$(frame)_(#$(frame.id))")

function setgeometry!(vis::Visualizer, mechanism::Mechanism, source::AbstractGeometrySource=Skeleton(false, true))
    elements = visual_elements(mechanism, source)
    setgeometry!(vis, mechanism, elements)
end

function setgeometry!(vis::Visualizer, mechanism::Mechanism, elements::AbstractVector{<:VisualElement})
    batch(vis) do v
        delete!(v)
        addgeometry!(vis, mechanism, elements)
    end
    nothing
end


apply_scaling(geometry::HyperSphere, scale::Vec) = HyperEllipsoid(center(geometry), scale) 
apply_scaling(geometry, scale) = error("Geometry $geometry has a non-unit scale: $scale, but I don't know how to transform it into a scaled geometry automatically.")

"""
The remote tree viewer interface has no notion of scale
(only a quaternion and translation), so we have to do some
monkey business to push the scaling into the geometry itself.

Currently, this only works for converting spheres into ellipsoids
and assumes the scaling is positive. 
"""
function remove_scaling(geometry, transform)
    H = [transform_deriv(transform, Vec(0., 0, 0)) transform(Vec(0., 0, 0));
    Vec(0, 0, 0, 1)']
    scale = Vec(norm(H[1:3, 1]), norm(H[1:3, 2]), norm(H[1:3, 3]))
    if norm(scale - Vec(1, 1, 1)) > 1e-3
        H[:,1] ./= scale[1]
        H[:,2] ./= scale[2]
        H[:,3] ./= scale[3]
        tform = AffineMap(H[1:3, 1:3], transform(Vec(0., 0, 0)))
        return apply_scaling(geometry, scale), tform
    else
        return geometry, transform
    end
end

function addgeometry!(vis::Visualizer, mechanism::Mechanism, elements::AbstractVector{<:VisualElement})
    frame_to_elements = Dict{CartesianFrame3D, Vector{VisualElement}}()
    for element in elements
        push!(get!(() -> VisualElement[], frame_to_elements, element.frame), element)
    end
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
                if haskey(frame_to_elements, frame)
                    found_geometry = true
                    for element in frame_to_elements[frame]
                        geometry, transform = remove_scaling(element.geometry, element.transform)
                        geomdata = GeometryData(geometry, element.color, transform)
                        addgeometry!(bodyvis[framename], geomdata)
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
    addgeometry!(vis, mechanism, [VisualElement(frame, geometry, RGBA{Float32}(1, 0, 0, 0.5), IdentityTransformation())])
end

function addgeometry!(vis::Visualizer, mechanism::Mechanism, frame::CartesianFrame3D; scale=1.0)
    addgeometry!(vis, mechanism, frame, Triad(scale, false))
end

function addgeometry!(vis::Visualizer, mechanism::Mechanism, point::Point3D; radius=0.03)
    addgeometry!(vis, mechanism, point.frame, HyperSphere(Point{3, Float64}(point.v), radius))
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

function Visualizer(mechanism::Mechanism, frame_geometries::Associative{CartesianFrame3D, Vector{GeometryData}},
                    prefix=[:robot1])
    vis = Visualizer()[prefix]
    setgeometry!(vis, mechanism, frame_geometries)
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
    elements = visual_elements(state.mechanism, Skeleton(show_inertias, randomize_colors))
    setgeometry!(vis, state.mechanism, elements)
    inspect!(state, vis)
end

inspect(mechanism::Mechanism, args...; kwargs...) =
    inspect!(MechanismState{Float64}(mechanism), args...; kwargs...)

@deprecate parse_urdf(urdf::AbstractString, mechanism::Mechanism; kwargs...) visual_elements(mechanism, URDFVisuals(urdf; kwargs...))

@deprecate create_geometry(mechanism; show_inertias=false, randomize_colors=true) visual_elements(mechanism, Skeleton(inertias=show_inertias, randomize_colors=randomize_colors))
