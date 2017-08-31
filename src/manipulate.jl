"""
joint_configuration maps the slider values to a joint configuration vector.
For a quaternion floating joint, this is nontrivial because we create three
sliders for the rotational degrees of freedom, which are used to represent
the rotation in exponential map form. Those three sliders then have to be
converted into a quaternion to set the joint configuration. We do this because
interacting with the four components of a quaternion is quite unintuitive.
"""
function joint_configuration(jointType::RigidBodyDynamics.QuaternionFloating,
                             sliders::NTuple{6, T}) where T
    q = collect(sliders)
    quat = Quat(RodriguesVec(q[1], q[2], q[3]))
    vcat([quat.w; quat.x; quat.y; quat.z], q[4:6])
end
joint_configuration(jointType::RigidBodyDynamics.OneDegreeOfFreedomFixedAxis,
                    sliders::NTuple{1, T}) where {T} = collect(sliders)
joint_configuration(jointType::RigidBodyDynamics.Fixed, sliders::Tuple{}) = []
num_sliders(jointType::RigidBodyDynamics.OneDegreeOfFreedomFixedAxis) = 1
num_sliders(jointType::RigidBodyDynamics.QuaternionFloating) = 6
num_sliders(jointType::RigidBodyDynamics.Fixed) = 0
num_sliders(joint::RigidBodyDynamics.Joint) = num_sliders(joint_type(joint))

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

    widgets = [Interact.widget(linspace(-pi, pi, 51), slider_names[i]) for i = 1:sum(num_sliders_per_joint)]
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
