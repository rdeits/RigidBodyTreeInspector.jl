module Manipulate

export manipulate!

using RigidBodyTreeInspector
using DrakeVisualizer
using RigidBodyDynamics
using RigidBodyDynamics: Bounds, position_bounds, lower, upper
using InteractBase: slider, Widget, observe, vbox
using DataStructures: OrderedDict

function remove_infs(b::Bounds, default=Float64(π))
    Bounds(isfinite(lower(b)) ? lower(b) : -default,
           isfinite(upper(b)) ? upper(b) : default)
end

slider_range(joint::Joint) = remove_infs.(position_bounds(joint))
function slider_range(joint::Joint{T, <: QuaternionFloating}) where {T}
    defaults = [1., 1, 1, 1, 10, 10, 10]
    remove_infs.(position_bounds(joint), defaults)
end

slider_labels(joint::Joint) = [string("q", i) for i in 1:num_positions(joint)]
slider_labels(joint::Joint{T, <:QuaternionFloating}) where {T} = ["rw", "rx", "ry", "rz", "x", "y", "z"]

function sliders(joint::Joint, values=zeros(num_positions(joint));
                 bounds=slider_range(joint),
                 labels=slider_labels(joint),
                 resolution=0.01, prefix="")
    map(bounds, labels, values) do b, label, value
        slider(lower(b):resolution:upper(b),
               value=value,
               label=string(prefix, label))
    end
end

function combined_observable(joint::Joint, sliders::AbstractVector)
    map(observe.(sliders)...) do args...
        q = vcat(args...)
        normalize_configuration!(q, joint)
        q
    end
end

function widget(joint::Joint{T, <:Fixed}, args...) where T
    Widget{:rbd_joint}()
end

function widget(joint::Joint, initial_value=zeros(num_positions(joint)); prefix=string(joint, '.'))
    s = sliders(joint, initial_value, prefix=prefix)
    keys = Symbol.(slider_labels(joint))
    w = Widget{:rbd_joint}(OrderedDict(zip(keys, s)))
    w.output = combined_observable(joint, s)
    w.layout = x -> vbox(s...)
    w
end

"""
    manipulate!(callback::Function, state::MechanismState)

Create Interact sliders to manipulate the state of the mechanism, and call
callback(state) each time a slider is changed. This mutates the state in-place.
"""
function manipulate!(callback::Function, state::MechanismState)
    joint_list = joints(state.mechanism)
    widgets = widget.(joint_list, configuration.(state, joint_list))
    keys = Symbol.(joint_list)
    w = Widget{:rbd_manipulator}(OrderedDict(zip(keys, widgets)))
    w.output = map(observe.(widgets)...) do signals...
        for i in 1:length(joint_list)
            if num_positions(joint_list[i]) > 0
                set_configuration!(state, joint_list[i], signals[i])
            end
        end
        callback(state)
    end
    w.layout = x -> vbox(widgets...)
    w
end

"""
    manipulate(callback::Function, mechanism::Mechanism)

Create Interact sliders to manipulate the state of the mechanism, and call
callback(state) each time a slider is changed. This constructs a new
MechanismState object to mutate internally.
"""
manipulate(callback::Function, mechanism::Mechanism) =
    manipulate(callback, MechanismState{Float64}(mechanism))

end


