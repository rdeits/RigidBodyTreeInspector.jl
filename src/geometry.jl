function rotation_between{T}(from::AbstractVector{T}, to::AbstractVector{T})
    from /= norm(from)
    to /= norm(to)
    costheta = dot(from, to)
    p = cross(from, to)
    sintheta = norm(p)
    if sintheta > 0
        axis = p / sintheta
        angle = atan2(sintheta, costheta)
        return AngleAxis(angle, axis[1], axis[2], axis[3])
    else
        return AngleAxis(0.0, 1, 0, 0)
    end
end

rotation_from_x_axis{T}(translation::AbstractVector{T}) = rotation_between(SVector{3, T}(1,0,0), translation)

function inertial_ellipsoid_dimensions(mass, axis_inertias)
    # Ix = m/5 (dy^2 + dz^2)
    # Iy = m/5 (dx^2 + dz^2)
    # Iz = m/5 (dx^2 + dy^2)
    #
    # let A = [0 1 1
    #          1 0 1
    #          1 1 0]
    # b = 5 / m * [Ix; Iy; Iz]
    # Then A \ b = [dx^2; dy^2; dz^2]
    #
    # This is only valid if the axis inertias obey the triangle inequalities:
    # Ix + Iy >= Iz
    # Ix + Iz >= Iy
    # Iy + Iz >= Ix

    # Ix - Iy = m/5 (dy^2 - dx^2)
    # Ix - Iy + Iz = m/5 (2*dy^2)
    # dy^2 = 0.5 (Ix - Iy + Iz) * 5/m


    squared_lengths = 0.5 * 5.0 / mass *
        [-axis_inertias[1] + axis_inertias[2] + axis_inertias[3];
          axis_inertias[1] - axis_inertias[2] + axis_inertias[3];
          axis_inertias[1] + axis_inertias[2] - axis_inertias[3]]

    for i = 1:3
        total_inertia_of_other_axes = zero(axis_inertias[1])
        for j = 1:3
            if i == j
                continue
            end
            total_inertia_of_other_axes += axis_inertias[j]
        end
        if axis_inertias[i] > total_inertia_of_other_axes
            error("Principal inertias $(axis_inertias) do not satisfy the triangle inequalities, so the equivalent inertial ellipsoid is not well-defined.")
        end
    end

    return √(squared_lengths)
end

function inertial_ellipsoid(body)
    inertia = get(body.inertia)
    com_frame = CartesianFrame3D("com")
    com_to_body = Transform3D(com_frame, body.frame, center_of_mass(inertia).v)
    spatial_inertia = transform(inertia, inv(com_to_body))
    e = eigfact(convert(Array, spatial_inertia.moment))
    principal_inertias = e.values
    axes = e.vectors
    axes[:,3] *= sign(dot(cross(axes[:,1], axes[:,2]), axes[:,3])) # Ensure the axes form a right-handed coordinate system
    radii = inertial_ellipsoid_dimensions(spatial_inertia.mass, principal_inertias)
    geometry = HyperEllipsoid{3, Float64}(zero(Point{3, Float64}), Vec{3, Float64}(radii))
    return geometry, AffineMap(axes, center_of_mass(inertia).v)
end

function create_geometry_for_translation{T}(translation::AbstractVector{T}, radius)
    Rx = rotation_from_x_axis(translation)
    geom_length = norm(translation)
    joint_to_geometry_origin = compose(compose(LinearMap(Rx),
                                               Translation(geom_length / 2, 0, 0)),
                                       LinearMap(AngleAxis(pi/2, 0, 1, 0)))
    # joint_to_geometry_origin = tformrotate(convert(Vector, a), theta) * tformtranslate([geom_length/2; 0; 0]) * tformrotate([0; pi/2; 0])
    return HyperCylinder{3, Float64}(geom_length, radius), joint_to_geometry_origin
end

function create_geometry(mechanism; show_inertias::Bool=false, randomize_colors::Bool=true)
    maximum_joint_to_joint_length = maximum([norm(mechanism.jointToJointTransforms[joint].trans) for joint in joints(mechanism)])
    box_width = 0.05 * maximum_joint_to_joint_length

    vis_data = OrderedDict{RigidBody, Link}()
    link_names = Set()
    for vertex in mechanism.toposortedTree
        if randomize_colors
            color = RGBA{Float64}(rand(3)..., 0.5)
        else
            color = RGBA{Float64}(1, 0, 0, 0.5)
        end
        body = vertex_data(vertex)
        geometries = Vector{GeometryData}()
        if show_inertias && !isnull(body.inertia) && get(body.inertia).mass >= 1e-3
        # if show_inertias && !isroot(mechanism, body) && body.inertia.mass >= 1e-3
            ellipsoid, tform = inertial_ellipsoid(body)
            push!(geometries, GeometryData(ellipsoid, tform, color))
        else
            push!(geometries, GeometryData(HyperSphere{3, Float64}(zero(Point{3, Float64}), box_width), IdentityTransformation(), color))
        end
        if !isroot(mechanism, body)
            for child in vertex.children
                joint = edge_to_parent_data(child)
                joint_to_joint = mechanism.jointToJointTransforms[joint]
                geom, tform = create_geometry_for_translation(joint_to_joint.trans, box_width/2)
                push!(geometries, GeometryData(geom, tform, color))
            end
        end
        unique_name = if body.name in link_names
            i = 1
            candidate = body.name * "_$(i)"
            while candidate in link_names
                i += 1
                candidate = body.name * "_$(i)"
            end
            candidate
        else
            body.name
        end
        push!(link_names, unique_name)
        vis_data[body] = Link(geometries, unique_name)
    end
    vis_data
end
