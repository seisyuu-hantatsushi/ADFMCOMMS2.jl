
module FFTView

using FFTW
using GLMakie

@enum WindowFunctions begin
    Hann
    Blackman
    BlackmanHaris
end

mutable struct Parameter
    inputSamplingRate::UInt64
    window::WindowFunctions
end

mutable struct ViewContext{T}
    running::Base.Threads.Atomic{Bool}
    parameter::Parameter
    fft_buf::Vector{T}
    fft_pos::Int
    win::Vector{Float32}
    tmp::Vector{ComplexF32}
    idx_lo::Int
    idx_hi::Int
    yobs::Observable{Vector{Float64}}
    screen::Union{Nothing, GLMakie.Screen}
    freq_view::Vector{Float64}
    main_task::Task
    worker::Task
    update_task::Task
    freeQ::Channel{Vector{ComplexF32}}
    fullQ::Channel{Tuple{Vector{ComplexF32}, Int}}
    resultQ::Channel{Vector{Float64}}
end

function hann_window_coeffs(ns::UnitRange{Int}, denom)
    return 0.5f0 .* (1 .- cos.(2f0 * Float32(π) .* Float32.(ns) ./ denom))
end

function blackman_window_coeffs(ns::UnitRange{Int}, denom)
    a0 = 0.42f0
    a1 = 0.5f0
    a2 = 0.08f0
    phase = 2f0 * Float32(π) .* Float32.(ns) ./ denom
    return a0 .- a1 .* cos.(phase) .+ a2 .* cos.(2f0 .* phase)
end

function blackman_haris_window_coeffs(ns::UnitRange{Int}, denom)
    a0 = 0.35875f0
        a1 = 0.48829f0
    a2 = 0.14128f0
    a3 = 0.01168f0
    phase = 2f0 * Float32(π) .* Float32.(ns) ./ denom
    return a0 .- a1 .* cos.(phase) .+ a2 .* cos.(2f0 .* phase) .- a3 .* cos.(3f0 .* phase)
end

function window_coeffs(fft_size::Int, window::WindowFunctions)
    ns = 0:fft_size - 1
    denom = Float32(fft_size - 1)
    if window == Hann
        return hann_window_coeffs(ns, denom)
    elseif window == Blackman
        return blackman_window_coeffs(ns, denom)
    else
        return blackman_haris_window_coeffs(ns, denom)
    end
end

function fftshift(v::AbstractVector)
    n = length(v)
    h = n ÷ 2
    return vcat(view(v, h + 1:n), view(v, 1:h))
end

function process_samples!(context::ViewContext{ComplexF32}, samples::AbstractVector{ComplexF32}, n::Integer)
    i = 1
    fft_size = length(context.fft_buf)
    while i <= n
        to_copy = min(fft_size - context.fft_pos + 1, n - i + 1)
        copyto!(context.fft_buf, context.fft_pos, samples, i, to_copy)
        context.fft_pos += to_copy
        i += to_copy
        if context.fft_pos > fft_size
            @inbounds for k in 1:fft_size
                context.tmp[k] = context.fft_buf[k] * context.win[k]
            end
            spec = abs.(fft(context.tmp))
            spec = fftshift(spec)
            spec_db = 20 .* log10.(spec ./ fft_size .+ eps(Float32))
            put!(context.resultQ, Float64.(spec_db[context.idx_lo:context.idx_hi]))
            context.fft_pos = 1
        end
    end
    return nothing
end

function CreateView(::Type{T}, inputSamplingRate::UInt64, numberOfFFTSampling::UInt64, window::WindowFunctions;
                    frame_size::Int = Int(numberOfFFTSampling),
                    poolsize::Int = 16,
                    title::AbstractString = "FFT View",
                    fmin::Float64 = -Float64(inputSamplingRate) / 2,
                    fmax::Float64 = Float64(inputSamplingRate) / 2,
                    window_size = (900, 480)) where {T}

    fft_size = Int(numberOfFFTSampling)
    fft_size < 2 && error("FFT size must be at least 2.")
    fmin >= fmax && error("Frequency range must satisfy fmin < fmax.")
    frame_size < 1 && error("Frame size must be at least 1.")
    poolsize < 1 && error("Pool size must be at least 1.")

    freqs = range(-Float64(inputSamplingRate) / 2, Float64(inputSamplingRate) / 2; length = Int(fft_size))
    idx_lo = max(1, searchsortedfirst(freqs, fmin))
    idx_hi = min(fft_size, searchsortedlast(freqs, fmax))
    idx_lo > idx_hi && error("Frequency range does not intersect FFT span.")
    freq_view = collect(freqs[idx_lo:idx_hi])

    fig = Figure(size = window_size)
    ax = Axis(fig[1, 1], xlabel = "Frequency (Hz)", ylabel = "Magnitude (dB)")
    tick_step = 10_000.0
    label_step = 100_000.0
    tick_vals = collect(range(fmin, fmax; step = tick_step))
    tick_labels = [mod(round(v), label_step) == 0 ? string(Int(round(v / 1000))) * "k" : "" for v in tick_vals]
    zero_idx = findfirst(==(0.0), tick_vals)
    if zero_idx !== nothing
        tick_labels[zero_idx] = "0"
    end
    ax.xticks = (tick_vals, tick_labels)
    yobs = Observable(fill(-120.0, length(freq_view)))
    lines!(ax, freq_view, yobs; linewidth = 1)
    ylims!(ax, -120, 20)
    main_task = current_task()
    renderloop = screen -> begin
        try
            GLMakie.renderloop(screen)
        catch e
            if e isa InterruptException
                Base.throwto(main_task, e)
                return
            end
            rethrow()
        end
    end
    screen = GLMakie.Screen(fig.scene; renderloop = renderloop)
    display(screen, fig)
    if screen !== nothing
        GLMakie.set_title!(screen, title)
    end

    freeQ = Channel{Vector{ComplexF32}}(poolsize)
    fullQ = Channel{Tuple{Vector{ComplexF32}, Int}}(poolsize)
    resultQ = Channel{Vector{Float64}}(poolsize)
    for _ in 1:poolsize
        put!(freeQ, Vector{ComplexF32}(undef, frame_size))
    end

    view = ViewContext(Base.Threads.Atomic{Bool}(true),
                       Parameter(inputSamplingRate, window),
                       Vector{T}(undef, fft_size),
                       1,
                       window_coeffs(fft_size, window),
                       Vector{ComplexF32}(undef, fft_size),
                       idx_lo,
                       idx_hi,
                       yobs,
                       screen,
                       freq_view,
                       main_task,
                       Task(() -> nothing),
                       Task(() -> nothing),
                       freeQ,
                       fullQ,
                       resultQ)
    view.update_task = @async update_task!(view)
    view.worker = Threads.@spawn task!(view)
    return view
end

function isopen(context::ViewContext)
    return context.screen === nothing || GLMakie.isopen(context.screen)
end

function task!(context::ViewContext{ComplexF32})
    try
        while context.running[]
            if context.screen !== nothing && !isopen(context)
                context.running[] = false
                break
            end
            if isready(context.fullQ)
                buf, n = take!(context.fullQ)
                process_samples!(context, buf, n)
                put!(context.freeQ, buf)
            else
                sleep(0.001)
            end
        end
    catch e
        if !(e isa InterruptException)
            println(e)
        end
    end
    return nothing
end

function update_task!(context::ViewContext{ComplexF32})
    try
        while context.running[]
            if isready(context.resultQ)
                context.yobs[] = take!(context.resultQ)
            else
                yield()
            end
        end
    catch e
        if !(e isa InterruptException)
            println(e)
        end
    end
    return nothing
end

function enqueue!(context::ViewContext{ComplexF32}, samples::AbstractVector{ComplexF32}, n::Integer)
    if !context.running[] || n <= 0
        return false
    end
    if !isready(context.freeQ)
        return false
    end
    buf = take!(context.freeQ)
    copyto!(buf, 1, samples, 1, n)
    put!(context.fullQ, (buf, n))
    return true
end

function stop!(context::ViewContext)
    println("FFTView.stop()")
    context.running[] = false
    wait(context.worker)
    wait(context.update_task)
    return nothing
end

end
