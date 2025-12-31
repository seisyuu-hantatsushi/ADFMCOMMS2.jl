using ADFMCOMMS2

const ADC_FS = 1_200_000
const BW = 800_000
const AUDIO_FS = 48_000
const DECIM = ADC_FS ÷ AUDIO_FS

function parse_si_hz(s::String)
    m = match(r"^([0-9]*\.?[0-9]+)([kKmMgGtTpP]?)$", s)
    m === nothing && error("Invalid frequency: $s")
    value = parse(Float64, m.captures[1])
    suffix = m.captures[2]
    scale = suffix == "" ? 1.0 :
        suffix in ("k", "K") ? 1e3 :
        suffix in ("M", "m") ? 1e6 :
        suffix in ("G", "g") ? 1e9 :
        suffix in ("T", "t") ? 1e12 :
        suffix in ("P", "p") ? 1e15 : 1.0
    return value * scale
end

function parse_args(args)
    carrier = nothing
    output = "fm.wav"
    seconds = nothing
    i = 1
    while i <= length(args)
        a = args[i]
        if a == "-c" || a == "--carrierFreq"
            i += 1
            i > length(args) && error("Missing value for $a")
            carrier = parse_si_hz(args[i])
        elseif a == "-o" || a == "--output"
            i += 1
            i > length(args) && error("Missing value for $a")
            output = args[i]
        elseif a == "-s" || a == "-t" || a == "--seconds"
            i += 1
            i > length(args) && error("Missing value for $a")
            seconds = parse(Float64, args[i])
        else
            error("Unknown argument: $a")
        end
        i += 1
    end
    carrier === nothing && error("Carrier frequency is required. Use -c/--carrierFreq (e.g. 77.8M).")
    return carrier, output, seconds
end

function write_wav(path::String, samples::Vector{Int16}, fs::Int)
    n = length(samples)
    data_bytes = n * sizeof(Int16)
    riff_size = 36 + data_bytes
    open(path, "w") do io
        write(io, "RIFF")
        write(io, UInt32(riff_size))
        write(io, "WAVE")
        write(io, "fmt ")
        write(io, UInt32(16))
        write(io, UInt16(1))
        write(io, UInt16(1))
        write(io, UInt32(fs))
        write(io, UInt32(fs * sizeof(Int16)))
        write(io, UInt16(sizeof(Int16)))
        write(io, UInt16(16))
        write(io, "data")
        write(io, UInt32(data_bytes))
        write(io, samples)
    end
end

mutable struct FMDemodState
    prev::ComplexF32
    acc::Float32
    acc_count::Int
    deemph_y::Float32
end

function FMDemodState()
    return FMDemodState(ComplexF32(1, 0), 0.0f0, 0, 0.0f0)
end

function demod_fm!(emit_sample, recv_buffer::AbstractVector{ComplexF32}, n::Int,
                   state::FMDemodState, deemph_alpha::Float32)
    @inbounds for i in 1:n
        x = recv_buffer[i]
        z = x * conj(state.prev)
        state.prev = x
        demod = atan(imag(z), real(z))
        state.acc += demod
        state.acc_count += 1
        if state.acc_count == DECIM
            sample = state.acc / DECIM
            state.deemph_y = deemph_alpha * state.deemph_y + (1 - deemph_alpha) * sample
            s = clamp(state.deemph_y * 0.5f0, -1f0, 1f0)
            if !emit_sample(Int16(round(s * 32767)))
                return false
            end
            state.acc = 0.0f0
            state.acc_count = 0
        end
    end
    return true
end

struct AudioOutput{T, W, C}
    io::T
    write_fn::W
    close_fn::C
end

function Base.write(out::AudioOutput, data)
    return out.write_fn(out.io, data)
end

function Base.close(out::AudioOutput)
    return out.close_fn(out.io)
end

function try_open_portaudio()
    try
        @eval import PortAudio
        pa = Base.invokelatest(getproperty, Main, :PortAudio)
        stream = Base.invokelatest(pa.PortAudioStream, 0, 1; samplerate=AUDIO_FS, frames_per_buffer=1024)
        write_fn = (io, data) -> Base.invokelatest(write, io, Float32.(data) .* (1 / 32768))
        close_fn = io -> Base.invokelatest(close, io)
        return AudioOutput(stream, write_fn, close_fn)
    catch
        return nothing
    end
end

function open_audio_output()
    pa = try_open_portaudio()
    if pa !== nothing
        return pa
    end
    cmds = (
        `aplay -q -f S16_LE -c 1 -r $(AUDIO_FS)`,
        `play -q -t raw -e signed-integer -b 16 -c 1 -r $(AUDIO_FS) -`,
    )
    for cmd in cmds
        try
            return AudioOutput(open(cmd, "w"), write, close)
        catch
        end
    end
    error("Failed to open audio output. Install PortAudio, `aplay` (ALSA), or `play` (sox).")
end

function main()
    Base.exit_on_sigint(false)
    carrier, output, seconds = parse_args(ARGS)
    to_audio = output == "audio"

    uri = ADFMCOMMS2.scan("ip")[1]
    adapter = ADFMCOMMS2.SDR_RxAdapter(uri, UInt64(round(carrier)), UInt32(ADC_FS), UInt32(BW), ComplexF32)
    recv_buffer = Vector{ComplexF32}(undef, length(adapter.dataBuffer.bufs[1]))

    target_samples = seconds === nothing ? 0 : Int(round(seconds * AUDIO_FS))
    audio = Vector{Int16}()
    audio_chunk = Vector{Int16}()
    if to_audio
        sizehint!(audio_chunk, 4096)
    else
        if seconds !== nothing
            sizehint!(audio, target_samples)
        end
    end
    audio_io = to_audio ? open_audio_output() : nothing

    state = FMDemodState()
    audio_failed = Ref(false)
    t_start = time()
    next_report = t_start + 1.0
    deemph_tau = 50e-6
    deemph_alpha = Float32(exp(-1.0 / (AUDIO_FS * deemph_tau)))

    emit_sample = if to_audio
        function (sample::Int16)
            push!(audio_chunk, sample)
            if length(audio_chunk) >= 4096
                try
                    write(audio_io, audio_chunk)
                catch e
                    @warn "Audio output failed" exception=(e, catch_backtrace())
                    audio_failed[] = true
                    return false
                end
                empty!(audio_chunk)
            end
            return !audio_failed[]
        end
    else
        function (sample::Int16)
            push!(audio, sample)
            if seconds !== nothing && length(audio) >= target_samples
                return false
            end
            return true
        end
    end

    try
        while true
            now = time()
            if now >= next_report
                println("elapsed $(round(now - t_start; digits=1)) s")
                next_report = now + 1.0
            end
            if seconds !== nothing && (now - t_start) >= seconds
                break
            end
            if !to_audio && seconds !== nothing && length(audio) >= target_samples
                break
            end
            n = ADFMCOMMS2.recv!(adapter, recv_buffer)
            if !demod_fm!(emit_sample, recv_buffer, n, state, deemph_alpha)
                break
            end
        end
    catch e
        if e isa InterruptException
            @warn "Interrupted by Ctrl-C"
        else
            rethrow()
        end
    finally
        Base.disable_sigint() do
            ADFMCOMMS2.close!(adapter)
            if to_audio
                if !audio_failed[]
                    if !isempty(audio_chunk)
                        write(audio_io, audio_chunk)
                    end
                    close(audio_io)
                end
            end
        end
        Base.exit_on_sigint(true)
    end

    if !to_audio
        write_wav(output, audio, AUDIO_FS)
    end
end

main()
