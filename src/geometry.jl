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

function inertial_ellipsoid(mechanism::Mechanism, body::RigidBody)
    inertia = spatial_inertia(body)
    com_frame = CartesianFrame3D("$(RigidBodyDynamics.name(body)) com")
    com_frame_to_inertia_frame = Transform3D(com_frame, inertia.frame, center_of_mass(inertia).v)
    add_body_fixed_frame!(mechanism, body, com_frame_to_inertia_frame)
    inertia = transform(inertia, inv(com_frame_to_inertia_frame))
    principal_inertias, axes = eig(Array(inertia.moment)) # StaticArrays.eig checks that the matrix is Hermitian with zero tolerance...
    axes[:,3] *= sign(dot(cross(axes[:,1], axes[:,2]), axes[:,3])) # Ensure the axes form a right-handed coordinate system
    radii = inertial_ellipsoid_dimensions(inertia.mass, principal_inertias)
    geometry = HyperEllipsoid{3, Float64}(zero(Point{3, Float64}), Vec{3, Float64}(radii))
    principal_axes_com_frame = CartesianFrame3D("$(RigidBodyDynamics.name(body)) com principal axes")
    inertia_frame_to_com_principal_axes_frame = Transform3D(inertia.frame, principal_axes_com_frame, inv(RotMatrix{3}(axes)), center_of_mass(inertia).v)
    add_body_fixed_frame!(mechanism, body, inv(inertia_frame_to_com_principal_axes_frame))
    @assert begin
        inertia_in_principal_axes = transform(inertia, find_fixed_transform(mechanism, inertia.frame, principal_axes_com_frame))
        moment = inertia_in_principal_axes.moment
        moment_diagonal = norm(moment - diagm(diag(moment)), Inf) < 1e-8
        com = center_of_mass(inertia_in_principal_axes).v
        com_zero = norm(com, Inf) < 1e-8
        moment_diagonal && com_zero
    end
    return geometry, principal_axes_com_frame
end

function create_geometry_for_translation(mechanism, body, joint, radius)
    joint_to_joint = find_fixed_transform(mechanism, joint.frameBefore,
                                          default_frame(mechanism, body))
    translation = joint_to_joint.trans
    Rx = rotation_from_x_axis(translation)
    geom_length = norm(translation)
    joint_to_geometry_origin = compose(compose(LinearMap(Rx),
                                               Translation(geom_length / 2, 0, 0)),
                                       LinearMap(AngleAxis(pi/2, 0, 1, 0)))
    frame = CartesianFrame3D("$(RigidBodyDynamics.name(body)) joint-to-joint translation")
    tform = Transform3D(frame, default_frame(mechanism, body),
                        joint_to_geometry_origin.m,
                        joint_to_geometry_origin.v)
    add_body_fixed_frame!(mechanism, body, tform)
    return HyperCylinder{3, Float64}(geom_length, radius), frame
end

function create_geometry(mechanism; show_inertias::Bool=false, randomize_colors::Bool=true)
    maximum_joint_to_joint_length = maximum([norm(mechanism.jointToJointTransforms[joint].trans) for joint in joints(mechanism)])
    box_width = 0.05 * maximum_joint_to_joint_length

    vis_data = OrderedDict{CartesianFrame3D, Link}()
    link_names = Set()
    for vertex in mechanism.toposortedTree
        if randomize_colors
            color = RGBA{Float64}(rand(3)..., 0.5)
        else
            color = RGBA{Float64}(1, 0, 0, 0.5)
        end
        body = vertex_data(vertex)
        if show_inertias && has_defined_inertia(body) && spatial_inertia(body).mass >= 1e-3
            ellipsoid, ellipsoid_frame = inertial_ellipsoid(mechanism, body)
            vis_data[ellipsoid_frame] = GeometryData(ellipsoid, IdentityTransformation(), color)
        else
            frame = default_frame(mechanism, body)
            vis_data[frame] = GeometryData(HyperSphere{3, Float64}(
                zero(Point{3, Float64}), box_width), IdentityTransformation(), color)
        end
        if !isroot(mechanism, body)
            for child in children(vertex)
                joint = edge_to_parent_data(child)
                geom, geom_frame = create_geometry_for_translation(mechanism,
                                                                   body,
                                                                   joint,
                                                                   box_width / 2)
                vis_data[geom_frame] = GeometryData(geom, IdentityTransformation(), color)
            end
        end
    end
    vis_data
end
