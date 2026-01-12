module ADFMCOMMS2

using Reexport

include("libIIO/libIIO.jl");
@reexport using .libIIO_jl;

include("libad9361.jl")

# constants
const TX_DEVICE_NAME  = "cf-ad9361-dds-core-lpc";
const RX_DEVICE_NAME  = "cf-ad9361-lpc";
const PHY_DEVICE_NAME = "ad9361-phy";

@enum GainMode begin
    MANUAL
    FAST_ATTACK
    SLOW_ATTACK
    HYBRID
    DEFAULT
end


export
    SDR_RxAdapter,
    SDR_TxAdapter

struct DataBuffer{T}
    bufs::Vector{Vector{T}}
    freeQ::Channel{Int}
    fullQ::Channel{Int}
end

function DataBuffer(::Type{T}, n::Integer, poolsize::Integer) where {T}
    bufs = [Vector{T}(undef, n) for _ in 1:poolsize]
    freeq = Channel{Int}(poolsize)
    fullq = Channel{Int}(poolsize)
    for i in 1:poolsize
        put!(freeq, i)
    end
    return DataBuffer{T}(bufs, freeq, fullq)
end

mutable struct RxConfig
    ch::UInt32;
    carrierFreq::UInt64;
    samplingRate::UInt32;
    bandwidth::UInt32;
    rfport::String;
    quadrature::Bool;
    rfdc::Bool;
    bbdc::Bool;
    gainMode::GainMode;
    gain::Float64;
    filter::Union{Nothing,String};
    auto_filter::Bool;
    sampleBufferSize::UInt64;
end

mutable struct AD936xRxContext
    context::Ptr{iio_context};
    phyDev::Ptr{iio_device};
    rxDev::Ptr{iio_device};
    iCh::Ptr{iio_channel};
    qCh::Ptr{iio_channel};
    rxBuf::Ptr{iio_buffer};
    rxConfig::RxConfig;
end

mutable struct SDR_RxAdapter{T}
    running::Base.Threads.Atomic{Bool};
    task::Union{Nothing, Task};
    dataBuffer::DataBuffer{T};
    rxCtx::AD936xRxContext;    
end

function rxTask!(adapter::SDR_RxAdapter{ComplexF32})
    println("start rxTask(ComplexF32)")
    rxCtx = adapter.rxCtx
    dataBuffer = adapter.dataBuffer
    try
        Base.disable_sigint() do
            while adapter.running[]
                read_size = C_iio_buffer_refill(rxCtx.rxBuf)
                if read_size < 0
                    if !adapter.running[]
                        break
                    end
                    error("Failed to refill rx buffer. ", Base.Libc.strerror(-1*read_size))
                end
                head = C_iio_buffer_first(rxCtx.rxBuf, rxCtx.iCh)
                tail = C_iio_buffer_end(rxCtx.rxBuf)
                if isready(dataBuffer.freeQ)
                    index = take!(dataBuffer.freeQ)
                    nbytes = Int(tail - head)
                    nsamples_i16 = nbytes ÷ sizeof(Int16)
                    ncomplex = nsamples_i16 ÷ 2
                    src_i16 = unsafe_wrap(Vector{Int16}, Ptr{Int16}(head), nsamples_i16; own=false)
                    buf = dataBuffer.bufs[index]
                    if length(buf) < ncomplex
                        resize!(buf, ncomplex)
                    end
                    scale = Float32(1 / 2048)
                    @inbounds for i in 1:ncomplex
                        i_s = src_i16[2i - 1]
                        q_s = src_i16[2i]
                        buf[i] = ComplexF32(Float32(i_s) * scale, Float32(q_s) * scale)
                    end
                    put!(dataBuffer.fullQ, index)
                else
                    error("Drop IQdata")
                end
                yield()
            end
        end
    catch e
        adapter.running[] = false
        println("ADFMCOMMS2 task:", e)
    end
    println("end rxTask")
    return nothing
end

function rxTask!(adapter::SDR_RxAdapter{Complex{Int16}})
    println("start rxTask(Complex{Int16})")
    rxCtx = adapter.rxCtx
    dataBuffer = adapter.dataBuffer
    try
        Base.disable_sigint() do
            while adapter.running[]
                read_size = C_iio_buffer_refill(rxCtx.rxBuf)
                if read_size < 0
                    if !adapter.running[]
                        break
                    end
                    error("Failed to refill rx buffer. ", Base.Libc.strerror(-1*read_size))
                end
                head = C_iio_buffer_first(rxCtx.rxBuf, rxCtx.iCh)
                tail = C_iio_buffer_end(rxCtx.rxBuf)
                if isready(dataBuffer.freeQ)
                    index = take!(dataBuffer.freeQ)
                    nbytes = Int(tail - head)
                    nsamples_i16 = nbytes ÷ sizeof(Int16)
                    ncomplex = nsamples_i16 ÷ 2
                    src_i16 = unsafe_wrap(Vector{Int16}, Ptr{Int16}(head), nsamples_i16; own=false)
                    buf = dataBuffer.bufs[index]
                    if length(buf) < ncomplex
                        resize!(buf, ncomplex)
                    end
                    @inbounds for i in 1:ncomplex
                        i_s = src_i16[2i - 1]
                        q_s = src_i16[2i]
                        buf[i] = Complex{Int16}(i_s, q_s)
                    end
                    put!(dataBuffer.fullQ, index)
                else
                    error("Drop IQdata")
                end
                yield()
            end
        end
    catch e
        adapter.running[] = false
        println("ADFMCOMMS2 task:", e)
    end
    println("end rxTask")
    return nothing
end


function rxTask!(adapter::SDR_RxAdapter{T}) where {T}
    println("start rxTask")
    try
        while adapter.running[]
            read_size = C_iio_buffer_refill(adapter.rxCtx.rxBuf)
            #println("read_size $read_size")
            yield()
        end
    catch e
        println(e)
    end
    println("end rxTask")
    return nothing
end

function setPhyCfg(phydev::Ptr{iio_device}, config::RxConfig)

    # configuration for RX Tranceivers
    # RX1 -> voltage0, RX2 -> voltage1

    label = string("voltage",(config.ch-1))
    rxphy_ch = C_iio_device_find_channel(phydev,label,false)
    if(rxphy_ch == C_NULL) error("Failed to find $label in $PHY_DEVICE_NAME."); end

    ret = C_iio_channel_attr_write(rxphy_ch, "rf_port_select", config.rfport);
    if (ret < 0) error("Failed to set rf_port_select of $label in $PHY_DEVICE_NAME. ", Base.Libc.strerror(-1*ret)); end

    ret = C_iio_channel_attr_write_longlong(rxphy_ch, "sampling_frequency", Int64(config.samplingRate));
    if (ret < 0) error("Failed to set sampling_frequency $config.samplingRate in $label at $PHY_DEVICE_NAME. ", Base.Libc.strerror(-1*ret)); end

    rxphy_lo = C_iio_device_find_channel(phydev, "altvoltage0", true)
    if(rxphy_lo == C_NULL) error("Failed to find altvoltage0 in $PHY_DEVICE_NAME."); end

    ret = C_iio_channel_attr_write_longlong(rxphy_lo, "frequency", Int64(config.carrierFreq))
    if (ret < 0) error("Failed to set lo frequency altvoltage0 at $PHY_DEVICE_NAME. ", Base.Libc.strerror(-1*ret)); end

    return Int32(0)

end

function getIQChannels(rxdev::Ptr{iio_device}, config::RxConfig)
    # I Channel of RX1 -> voltage0 in cf-ad9361-lpc
    # Q Channel of RX1 -> voltage1 in cf-ad9361-lpc
    # I Channel of RX2 -> voltage2 in cf-ad9361-lpc
    # Q Channel of RX2 -> voltage3 in cf-ad9361-lpc
    i_index = (config.ch-1)*2
    q_index = (config.ch-1)*2 + 1

    label = string("voltage", i_index)
    i_ch = C_iio_device_find_channel(rxdev, label, false);
    if i_ch == C_NULL
        label = string("altvoltage", i_index)
        i_ch = C_iio_device_find_channel(rxdev, label, false);
    end
    if(i_ch == C_NULL) error("Failed to find i channel in $RX_DEVICE_NAME."); end


    label = string("voltage", q_index)
    q_ch = C_iio_device_find_channel(rxdev, label, false);
    if q_ch == C_NULL
        label = string("altvoltage", q_index)
        q_ch = C_iio_device_find_channel(rxdev, label, false);
    end
    if(q_ch == C_NULL) error("Failed to find q channel in $RX_DEVICE_NAME."); end

    return i_ch,q_ch
end

function openAD936x(uri::String, config::RxConfig)

    context = C_iio_create_context_from_uri(uri)
    if(context == C_NULL) error("Failed to create IIO context from uri $uri"); end
    if(C_iio_context_get_devices_count(context) == 0) error("No device found in context from uri $uri"); end

    phydev = C_iio_context_find_device(context, PHY_DEVICE_NAME)
    if(phydev == C_NULL) error("Failed to find ", PHY_DEVICE_NAME, "."); end

    rxdev =  C_iio_context_find_device(context, RX_DEVICE_NAME);
    if(rxdev == C_NULL) error("Failed to find ", RX_DEVICE_NAME, "."); end

    ret = ad9361_set_bb_rate(phydev, config.samplingRate)
    println("set FIR filter result = $ret");
    if (ret < 0) error("Failed to set FIR filter"); end

    setPhyCfg(phydev, config)

    i_ch,q_ch = getIQChannels(rxdev, config);

    C_iio_channel_enable(i_ch);
    C_iio_channel_enable(q_ch);

    # none-cyclic buffer
    rxBuf = C_iio_device_create_buffer(rxdev, config.sampleBufferSize, false);
    if(rxBuf == C_NULL) error("Failed to create a buffer for IQData."); end
    
    return AD936xRxContext(context, phydev, rxdev, i_ch, q_ch, rxBuf, config)
end

function closeAD936x!(context::AD936xRxContext)
    C_iio_channel_disable(context.iCh)
    C_iio_channel_disable(context.qCh)
    C_iio_buffer_destroy(context.rxBuf)
    C_iio_context_destroy(context.context)
end


function SDR_RxAdapter(uri::String, frequency::UInt64, samplerate::UInt32, bandwidth::UInt32, ::Type{T};
                       tranceiver_ch::UInt32=UInt32(1), rfport::String="A_BALANCED", quadrature::Bool = true,
                       sampleBufferSize::UInt64 = UInt64(64*1024),
                       ) where {T<:Union{ComplexF32,Complex{Int16}}}
                                       
    rxcfg = RxConfig(tranceiver_ch, frequency, samplerate, bandwidth,
                     rfport, quadrature, true, true, SLOW_ATTACK, 0.0, nothing, true, sampleBufferSize)
    
    adapter = SDR_RxAdapter{T}(Base.Threads.Atomic{Bool}(true),
                               nothing,
                               DataBuffer(T, sampleBufferSize, 128),
                               openAD936x(uri,rxcfg))
    finalizer(adapter) do x
        try
            close!(x)
        catch
        end
    end
    
    return adapter
end

function SDR_RxAdapter(::String, ::UInt64, ::UInt32, ::UInt32, ::Type{T};
                       tranceiver_ch::UInt32=UInt32(1), rfport::String="A_BALANCED", quadrature::Bool = true,
                       sampleBufferSize::UInt64 = UInt64(64*1024),
                       ) where {T}
    error("Unsupported sample type: $T (only ComplexF32 or Complex{Int16})")
end

function start!(adapter::SDR_RxAdapter{T}) where{T}
    adapter.task = Threads.@spawn rxTask!(adapter)
    return nothing
end

function stop!(adapter::SDR_RxAdapter{T}) where {T}
    if !adapter.running[]
        return nothing
    end

    adapter.running[] = false
    C_iio_buffer_cancel(adapter.rxCtx.rxBuf)
    if adapter.task !== nothing
        Base.disable_sigint() do
            try
                wait(adapter.task)
            catch e
                if !(e isa InterruptException)
                    rethrow()
                end
            end
        end
    end
end

function recv!(adapter::SDR_RxAdapter, recv_buffer::AbstractVector{T}) where{T}

    if !adapter.running[]
        eprintln("adapter is closed\n")
        return -1
    end

    dataBuffer = adapter.dataBuffer
    index = take!(dataBuffer.fullQ)
    src = dataBuffer.bufs[index]
    if length(recv_buffer) < length(src)
        put!(dataBuffer.freeQ, index)
        error("recv_buffer is too small: $(length(recv_buffer)) < $(length(src))")
    end
    n = length(src)
    copyto!(recv_buffer, 1, src, 1, n)
    put!(dataBuffer.freeQ, index)
    return n

end

function close!(adapter::SDR_RxAdapter{T}) where {T}

    stop!(adapter)

    closeAD936x!(adapter.rxCtx)
    
    return nothing
end

function SamplingFrameSize(adapter::SDR_RxAdapter)
    return adapter.rxCtx.rxConfig.sampleBufferSize
end


# ------------------------ #
# --- Helper functions --- #
# ------------------------ #
"""
    scan(backend[, infoIndex, doPrint])

Returns a device URI.

# Arguments
- `backend::Union{Nothing,String}` : the backend to scan (local, xml, ip, usb).
- `doPrint::Bool=true` : toggles console printing of the uri.

# Returns
- `uri::Vector{String}` : A vector of devices URI.

[C equivalent](https://analogdevicesinc.github.io/libiio/master/libiio/iio-monitor_8c-example.html#_a15)
"""

function scan(backend::Union{Nothing,String}, doPrint=true)

    # Check if backend is available and create scan context
    if backend !== nothing
        C_iio_has_backend(backend) || error("Specified backend $backend is not available");
    end
    scan_context = C_iio_create_scan_context(backend)

    # Getting metadata from scan
    info = Ref{Ptr{Ptr{iio_context_info}}}(0);
    ret = C_iio_scan_context_get_info_list(scan_context, info);
    
    # Get info list returns an error, shortcut 
    (ret < 0) && (return [""])

    # Get backendsusb address
    uri = Vector{String}(undef,0)
    if ret < 0
        C_iio_context_info_list_free(info[]);
        C_iio_scan_context_destroy(scan_context);
        #error("iio_scan_context_get_info_list failed with error $ret :\n", C_iio_strerror(ret));
        @warn "[$backend Backend] iio_scan_context_get_info_list failed with error $(C_iio_strerror(ret))"
        uri = [""]
    elseif ret == 0
        (doPrint) && (@info "No $backend device found");
        uri = [""]
    else
        for deviceIndex in 1:ret
            loaded_info = unsafe_load(info[], deviceIndex);
            description = C_iio_context_info_get_description(loaded_info);
            currUri = C_iio_context_info_get_uri(loaded_info);
            (doPrint) && (@info "Found $ret device(s) with $backend backend.\nSelected $description [$currUri]");
            push!(uri,currUri)
        end
    end
    C_iio_context_info_list_free(info[]);
    C_iio_scan_context_destroy(scan_context);

    return uri;

end



""" 
   print(adapter)
   Print the current radio configuration 
# Arguments
- `adapter::SDR_RxAdapter` : the radio to receive the samples from, and the structure storing those samples.

# Returns
- A string with the different configuration aspects
"""

function Base.print(adapter::SDR_RxAdapter)
    println("ADFMCOMMS2.SDR_RxAdapter.");    
end

end
