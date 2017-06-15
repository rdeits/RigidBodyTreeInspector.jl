type DrakeVisualizerSink <: OdeIntegrators.OdeResultsSink
    vis::Visualizer
    min_wall_Δt::Float64
    last_update_wall_time::Float64

    function DrakeVisualizerSink(vis::Visualizer; max_fps::Float64 = 60.)
        new(vis, 1 / max_fps, -Inf)
    end
end

function OdeIntegrators.initialize(sink::DrakeVisualizerSink, t, state)
    sink.last_update_wall_time = -Inf
    OdeIntegrators.process(sink, t, state)
end

function OdeIntegrators.process(sink::DrakeVisualizerSink, t, state)
    wall_Δt = time() - sink.last_update_wall_time
    if wall_Δt > sink.min_wall_Δt
        settransform!(sink.vis, state)
        sink.last_update_wall_time = time()
    end
    nothing
end
