using RigidBodyDynamics: Bounds, position_bounds, lower, upper

joint_configuration(joint_type::JointType, sliders::NTuple) = collect(sliders)
num_sliders(joint::RigidBodyDynamics.Joint) = num_sliders(joint_type(joint))
num_sliders(joint_type::JointType) = num_positions(joint_type)
slider_range(joint::RigidBodyDynamics.Joint) = position_bounds(joint)

"""
joint_configuration maps the slider values to a joint configuration vector.
For a quaternion floating joint, this is nontrivial because we create three
sliders for the rotational degrees of freedom, which are used to represent
the rotation in exponential map form. Those three sliders then have to be
converted into a quaternion to set the joint configuration. We do this because
interacting with the four components of a quaternion is quite unintuitive.
"""
function joint_configuration(joint_type::RigidBodyDynamics.QuaternionFloating,
                             sliders::NTuple{6, T}) where T
    q = collect(sliders)
    quat = Quat(RodriguesVec(q[1], q[2], q[3]))
    vcat([quat.w; quat.x; quat.y; quat.z], q[4:6])
end
num_sliders(joint_type::RigidBodyDynamics.QuaternionFloating) = 6
slider_range(joint::RigidBodyDynamics.QuaternionFloating) = fill(Bounds(-π, π), 3)

"""
    manipulate!(callback::Function, state::MechanismState)

Create Interact sliders to manipulate the state of the mechanism, and call
callback(state) each time a slider is changed. This mutates the state in-place.
"""
function manipulate!(callback::Function, state::MechanismState)
    mechanism = state.mechanism
    mech_joints = tree_joints(mechanism)
    num_sliders_per_joint = map(num_sliders, mech_joints)
    slider_names = String[]
    for (i, joint) in enumerate(mech_joints)
        for j in 1:num_sliders_per_joint[i]
            push!(slider_names, "$(joint.name).$(j)")
        end
    end
    slider_ranges = [r for joint in mech_joints for r in slider_range(joint)]

    widgets = [Interact.widget(linspace(lower(slider_ranges[i]), upper(slider_ranges[i]), 51), slider_names[i]) for i = 1:sum(num_sliders_per_joint)]
    foreach(display, widgets)
    foreach(map(Interact.signal, widgets)...) do q...
        slider_index = 1
        for (i, joint) in enumerate(mech_joints)
            configuration(state, joint)[:] = joint_configuration(joint_type(joint), q[slider_index:(slider_index+num_sliders_per_joint[i]-1)])
            slider_index += num_sliders_per_joint[i]
        end
        setdirty!(state)
        callback(state)
    end
    return
end

"""
    manipulate(callback::Function, mechanism::Mechanism)

Create Interact sliders to manipulate the state of the mechanism, and call
callback(state) each time a slider is changed. This constructs a new
MechanismState object to mutate internally.
"""
manipulate(callback::Function, mechanism::Mechanism) =
    manipulate(callback, MechanismState{Float64}(mechanism))
