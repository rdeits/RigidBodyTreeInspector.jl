rotation_from_x_axis{T}(translation::AbstractVector{T}) = Rotations.rotation_between(SVector{3, T}(1,0,0), translation)

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
    add_frame!(body, com_frame_to_inertia_frame)
    inertia = transform(inertia, inv(com_frame_to_inertia_frame))
    principal_inertias, axes = eig(Array(inertia.moment)) # StaticArrays.eig checks that the matrix is Hermitian with zero tolerance...
    axes[:,3] *= sign(dot(cross(axes[:,1], axes[:,2]), axes[:,3])) # Ensure the axes form a right-handed coordinate system
    radii = inertial_ellipsoid_dimensions(inertia.mass, principal_inertias)
    geometry = HyperEllipsoid{3, Float64}(zero(Point{3, Float64}), Vec{3, Float64}(radii))
    principal_axes_com_frame = CartesianFrame3D("$(RigidBodyDynamics.name(body)) com principal axes")
    inertia_frame_to_com_principal_axes_frame = Transform3D(inertia.frame, principal_axes_com_frame, inv(RotMatrix{3}(axes)), center_of_mass(inertia).v)
    add_frame!(body, inv(inertia_frame_to_com_principal_axes_frame))
    @assert begin
        inertia_in_principal_axes = transform(inertia, fixed_transform(mechanism, inertia.frame, principal_axes_com_frame))
        moment = inertia_in_principal_axes.moment
        moment_diagonal = norm(moment - diagm(diag(moment)), Inf) < 1e-8
        com = center_of_mass(inertia_in_principal_axes).v
        com_zero = norm(com, Inf) < 1e-8
        moment_diagonal && com_zero
    end
    return geometry, principal_axes_com_frame
end

function create_frame_to_frame_geometry(mechanism, body, frame1, frame2, radius)
    joint_to_joint = fixed_transform(mechanism, frame1, frame2)
    trans = translation(joint_to_joint)
    Rx = rotation_from_x_axis(trans)
    geom_length = norm(trans)
    joint_to_geometry_origin = compose(compose(LinearMap(Rx),
                                               Translation(geom_length / 2, 0, 0)),
                                       LinearMap(AngleAxis(pi/2, 0, 1, 0)))
    frame = CartesianFrame3D("$(RigidBodyDynamics.name(body)) joint-to-joint translation")
    tform = Transform3D(frame, joint_to_joint.to,
                        joint_to_geometry_origin.m,
                        joint_to_geometry_origin.v)
    add_frame!(body, tform)
    HyperCylinder{3, Float64}(geom_length, radius), frame
end

function maximum_link_length{T}(body_fixed_joint_frames::Dict{RigidBody{T}, Vector{CartesianFrame3D}})
    result = zero(T)
    for (body, joint_frames) in body_fixed_joint_frames
        for framei in joint_frames, framej in joint_frames
            transform = fixed_transform(body, framei, framej)
            result = max(result, norm(translation(transform)))
        end
    end
    result
end

function create_geometry(mechanism; show_inertias::Bool=false, randomize_colors::Bool=true)
    body_fixed_joint_frames = Dict(body => begin
        [map(frame_before, out_joints(body, mechanism)); map(frame_after, in_joints(body, mechanism))]
    end for body in bodies(mechanism))

    box_width = 0.05 * maximum_link_length(body_fixed_joint_frames)
    vis_data = OrderedDict{CartesianFrame3D, Vector{GeometryData}}()
    link_names = Set()

    for body in bodies(mechanism)
        if randomize_colors
            color = RGBA{Float64}(rand(3)..., 0.5)
        else
            color = RGBA{Float64}(1, 0, 0, 0.5)
        end
        if show_inertias && has_defined_inertia(body) && spatial_inertia(body).mass >= 1e-3
            ellipsoid, ellipsoid_frame = inertial_ellipsoid(mechanism, body)
            vis_data[ellipsoid_frame] = [GeometryData(ellipsoid, color)]
        else
            for joint in out_joints(body, mechanism)
                vis_data[frame_before(joint)] = [GeometryData(HyperSphere{3, Float64}(
                    zero(Point{3, Float64}), box_width), color)]
            end
        end
        frames = body_fixed_joint_frames[body]
        for (i, framei) in enumerate(frames)
            for j = i + 1 : length(frames)
                framej = frames[j]
                geom, geom_frame = create_frame_to_frame_geometry(mechanism, body, framei, framej, box_width / 2)
                vis_data[geom_frame] = [GeometryData(geom, color)]
            end
        end
    end
    vis_data
end
