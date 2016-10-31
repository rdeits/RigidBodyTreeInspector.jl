Base.one(::Type{Array{Float64,1}}) = 1.

normalize_configuration!(jointType::JointType, q) = nothing
function normalize_configuration!(jointType::QuaternionFloating, q)
    n = norm(q[1:4])
    for i = 1:4
        q[i] /= n
    end
end

"""
    animate(vis::Visualizer, mechanism::Mechanism{Float64},
            times::Vector{Float64},
            configurations::Vector{Vector{Float64}};
            fps::Float64=30, realtimerate::Float64=1)

Animate the given mechanism passing through a time-coded series of
configurations by linearly interpolating the configuration vectors.
"""
function animate(vis::Visualizer, mechanism::Mechanism{Float64},
                 times::Vector{Float64},
                 configurations::Vector{Vector{Float64}};
                 fps::Float64 = 30., realtimerate::Float64 = 1.)
    state = MechanismState(Float64, mechanism)
    dt = 1. / fps
    interpolated_configurations = interpolate((times,), configurations, Gridded(Linear()))
    for t in times[1] : dt : times[end]
        tic()
        q = interpolated_configurations[t]
        for joint in joints(mechanism)
            q_range = mechanism.qRanges[joint]
            q_joint = q[q_range]
            normalize_configuration!(joint.jointType, q_joint)
            configuration(state, joint)[:] = q_joint
        end
        setdirty!(state)
        draw(vis, state)
        sleep(max(dt - toq(), 0) / realtimerate)
    end
end

animate(mechanism::Mechanism, times::Vector{Float64},
        configurations::Vector{Vector{Float64}}) =
    animate(Visualizer(mechanism), mechanism, times, configurations)