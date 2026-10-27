# Shape-preserving cubic Hermite interpolation on uniformly spaced knots.
# Derivatives are with respect to normalized phase, not seconds.
function pcc_monotone_profile(values)
    y = Float64.(values)
    length(y) >= 2 && all(isfinite, y) || error("轨迹插值至少需要两个有限采样值")
    n = length(y)
    h = 1.0 / (n - 1)
    d = diff(y) ./ h
    m = zeros(n)
    for i in 2:n-1
        if d[i-1] * d[i] > 0
            m[i] = 2 * d[i-1] * d[i] / (d[i-1] + d[i])
        end
    end
    # Zero endpoint slopes allow an exact hold at approach/shape boundaries.
    function evaluate(s::Real)
        isfinite(s) || error("轨迹相位必须有限")
        u = clamp(Float64(s), 0.0, 1.0)
        k = min(floor(Int, u * (n - 1)) + 1, n - 1)
        w = (u - (k - 1) * h) / h
        value = (2w^3 - 3w^2 + 1)*y[k] + (w^3 - 2w^2 + w)*h*m[k] +
                (-2w^3 + 3w^2)*y[k+1] + (w^3 - w^2)*h*m[k+1]
        derivative = ((6w^2 - 6w)*y[k] + (3w^2 - 4w + 1)*h*m[k] +
                      (-6w^2 + 6w)*y[k+1] + (3w^2 - 2w)*h*m[k+1]) / h
        (; value, derivative)
    end
    evaluate
end

# Smoothstep velocity ramps, constant velocity in between. Integral is exactly 1.
function pcc_phase_profile(t::Real, duration::Real, ramp::Real)
    all(isfinite, (t, duration, ramp)) && duration > 0 && ramp > 0 ||
        error("轨迹时间须有限，时长和启停时间须大于零")
    r = min(Float64(ramp), Float64(duration)/2)
    v = 1 / (duration - r)
    if t <= 0
        return (phase=0.0, rate=0.0)
    elseif t >= duration
        return (phase=1.0, rate=0.0)
    elseif t < r
        u = t/r
        return (phase=v*r*(u^3 - u^4/2), rate=v*(3u^2 - 2u^3))
    elseif t > duration-r
        tail = pcc_phase_profile(duration-t, duration, ramp)
        return (phase=1-tail.phase, rate=tail.rate)
    end
    (phase=v*(t-r/2), rate=v)
end

function pcc_reference_evaluator(bend, axial, shape_start_index, shape;
                                 approach_seconds, transition_pause_s,
                                 trajectory_seconds, start_slow_s)
    channels = (bend..., axial...)
    approach = map(v -> pcc_monotone_profile(v[1:shape_start_index]), channels)
    shape_count = length(channels[1]) - shape_start_index + 1
    edges = shape == :rectangle ? 4 : 1
    shape_count % edges == 0 || error("矩形采样点须均分为四条边")
    per_edge = shape_count ÷ edges
    profiles = [map(v -> pcc_monotone_profile(
        v[shape_start_index+(edge-1)*per_edge:shape_start_index+edge*per_edge-1]),
        channels) for edge in 1:edges]
    track_start = approach_seconds + transition_pause_s
    function at(t::Real)
        isfinite(t) || error("采样时间必须有限")
        if t < approach_seconds
            phase = clamp(t/approach_seconds, 0.0, 1.0)
            rate = t <= 0 ? 0.0 : 1/approach_seconds
            selected = approach
        elseif t < track_start
            phase, rate = 1.0, 0.0
            selected = approach
        else
            elapsed = clamp(t-track_start, 0.0, trajectory_seconds)
            edge_duration = trajectory_seconds/edges
            edge = min(floor(Int, elapsed/edge_duration)+1, edges)
            timing = pcc_phase_profile(elapsed-(edge-1)*edge_duration,
                                       edge_duration, start_slow_s)
            phase, rate = timing.phase, timing.rate
            selected = profiles[edge]
        end
        samples = map(f -> f(phase), selected)
        (bend=(samples[1].value, samples[2].value),
         axial=(samples[3].value, samples[4].value),
         bend_rate=(samples[1].derivative*rate, samples[2].derivative*rate),
         axial_rate=(samples[3].derivative*rate, samples[4].derivative*rate))
    end
    at
end
