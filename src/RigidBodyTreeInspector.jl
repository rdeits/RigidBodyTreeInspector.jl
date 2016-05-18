module RigidBodyTreeInspector

using RigidBodyDynamics
import Quaternions: axis, angle, qrotation
import DrakeVisualizer: Visualizer, draw, Link, GeometryData
import FixedSizeArrays: Vec
import AffineTransforms: AffineTransform, tformrotate, tformtranslate
import GeometryTypes: HyperRectangle
import DataStructures: OrderedDict
import Interact
import Base: convert

export create_geometry, inspect, Visualizer, draw

function rotation_from_x_axis{T}(translation::Vec{3, T})
    xhat = Vec{3, T}(1, 0, 0)
    v = translation / norm(translation)
    costheta = dot(xhat, v)
    p = cross(xhat, v)
    sintheta = norm(p)
    axis = p / sintheta
    angle = atan2(sintheta, costheta)
    qrotation(convert(Vector, axis), angle)
end

function create_geometry(mechanism, box_width=0.05)
    vis_data = OrderedDict{RigidBody, Link}()
    for vertex in mechanism.toposortedTree[2:end]
        geometries = [GeometryData(HyperRectangle(Vec(-0.05, -0.05, -0.05), Vec(0.1, 0.1, 0.1)))]
        for child in vertex.children
            joint = child.edgeToParentData
            joint_to_joint = mechanism.jointToJointTransforms[joint]
            q = rotation_from_x_axis(joint_to_joint.trans)
            joint_to_geometry_origin = tformrotate(axis(q), angle(q))
            geom_length = norm(joint_to_joint.trans)
            push!(geometries, GeometryData(HyperRectangle(Vec(0., -box_width/2, -box_width/2), Vec(geom_length, box_width, box_width)), joint_to_geometry_origin))
        end
        vis_data[vertex.vertexData] = Link(geometries, vertex.vertexData.frame.name)
    end
    vis_data
end

function Visualizer(mechanism::Mechanism)
    vis_data = create_geometry(mechanism)
    Visualizer(collect(values(vis_data)))
end

convert(::Type{AffineTransform}, T::Transform3D) = tformtranslate(convert(Vector, T.trans)) * tformrotate(axis(T.rot), angle(T.rot))

function draw(vis::Visualizer, state::MechanismState)
    bodies = [v.vertexData for v in state.mechanism.toposortedTree[2:end]]
    origin_transforms = map(body -> convert(AffineTransform, transform_to_root(state, body.frame)), bodies)
    draw(vis, origin_transforms)
end

function joint_configuration{T}(jointType::RigidBodyDynamics.QuaternionFloating, sliders::NTuple{6, T})
    q = collect(sliders)
    quat = qrotation(q[1:3])

    vcat([quat.s; quat.v1; quat.v2; quat.v3], q[4:6])
end
joint_configuration{T}(jointType::RigidBodyDynamics.OneDegreeOfFreedomFixedAxis, sliders::NTuple{1, T}) = collect(sliders)
num_sliders(jointType::RigidBodyDynamics.OneDegreeOfFreedomFixedAxis) = 1
num_sliders(jointType::RigidBodyDynamics.QuaternionFloating) = 6
num_sliders(joint::RigidBodyDynamics.Joint) = num_sliders(joint.jointType)

function inspect(mechanism)
    vis = Visualizer(mechanism)
    state = MechanismState(Float64, mechanism)
    mech_joints = [v.edgeToParentData for v in mechanism.toposortedTree[2:end]]
    num_sliders_per_joint = map(num_sliders, mech_joints)
    slider_names = ASCIIString[]
    for (i, joint) in enumerate(mech_joints)
        for j in 1:num_sliders_per_joint[i]
            push!(slider_names, "$(joint.name).$(j)")
        end
    end
    widgets = [Interact.widget(linspace(-pi, pi), slider_names[i]) for i = 1:sum(num_sliders_per_joint)]
    map(display, widgets)
    map((q...) -> begin
        slider_index = 1
        for (i, joint) in enumerate(mech_joints)
            state.q[joint][:] = joint_configuration(joint.jointType, q[slider_index:(slider_index+num_sliders_per_joint[i]-1)])
            slider_index += num_sliders_per_joint[i]
        end
        setdirty!(state)
        draw(vis, state)
        end, [Interact.signal(w) for w in widgets]...)
end

end
