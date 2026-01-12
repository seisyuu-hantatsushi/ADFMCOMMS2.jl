
include("fft_view.jl")

using ADFMCOMMS2
using GLMakie

const ADC_SamplingRate   = 1_200_000 # A/D Sampling Rate 1.2MHz
const BandWidth          =   800_000 # Band width 800KHz
const Audio_SamplingRate =    48_000 # Demodulated Audio Sampling rate

function parse_si_hz(s::String)
    m = match(r"^([+-]?[0-9]*\.?[0-9]+)([kKmMgGtTpP]?)$", s)
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
    uri = nothing

    i = 1
    while i <= length(args)
        a = args[i]
        if a == "-c" || a == "--carrierFreq"
            i += 1
            i > length(args) && error("Missing value for $a")
            carrier = parse_si_hz(args[i])
        elseif a == "-i" || a == "--uri"
            i += 1
            i > length(args) && error("Missing value for $a")
            uri = args[i]
        else
            error("Unknown argument: $a")
        end
        i += 1
    end
    carrier === nothing && error("Carrier frequency is required. Use -c/--carrierFreq (e.g. 77.8M).")
    return carrier, uri
end

function main()

    carrier, uri = parse_args(ARGS)

    if uri === nothing
        uri = ADFMCOMMS2.scan("ip")[1]
        if isempty(uri)
            error("No SDR URI found via scan(\"ip\"). Use -i/--uri (e.g. ip:192.168.10.90).")
        end
    end

    adapter = ADFMCOMMS2.SDR_RxAdapter(uri,
                                       UInt64(round(carrier)),
                                       UInt32(ADC_SamplingRate),
                                       UInt32(BandWidth),
                                       ComplexF32)
    recv_buffer = Vector{ComplexF32}(undef, ADFMCOMMS2.SamplingFrameSize(adapter))
    view = FFTView.CreateView(ComplexF32,
                              UInt64(ADC_SamplingRate),
                              UInt64(8196),
                              FFTView.Hann;
                              frame_size = length(recv_buffer),
                              fmin = -BandWidth / 2,
                              fmax = BandWidth / 2)
    atexit(() -> begin
        view.running[] = false
        FFTView.stop!(view)
        Base.disable_sigint() do
            ADFMCOMMS2.close!(adapter)
        end
    end)

    while view.running[]
        if !FFTView.isopen(view)
            view.running[] = false
            break
        end
        recv_size = ADFMCOMMS2.recv!(adapter, recv_buffer)
        if recv_size < 0
            error("RF Receive Error")
        end
        FFTView.enqueue!(view, recv_buffer, recv_size)
    end
    view.running[] = false

end

main()
