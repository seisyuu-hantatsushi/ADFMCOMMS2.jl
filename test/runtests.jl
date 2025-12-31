using ADFMCOMMS2
using Dates
using Printf
using Test

function format_si(value)
    prefixes = ("", "k", "M", "G", "T", "P")
    scaled = float(value)
    idx = 1
    while abs(scaled) >= 1000 && idx < length(prefixes)
        scaled /= 1000
        idx += 1
    end
    return scaled, prefixes[idx]
end

@testset "libIIO_jl" begin
    include("test_toplevel.jl")
end

@testset "ADFMCOMMS2.jl" begin

    uri = ADFMCOMMS2.scan("ip")[1]

    adapter = ADFMCOMMS2.SDR_RxAdapter(uri, UInt64(77.8e6), UInt32(1.2e6), UInt32(800e3), ComplexF32)
    recv_buffer = Vector{ComplexF32}(undef, length(adapter.dataBuffer.bufs[1]))
    total_bytes = 0
    io = open("test_iq.dat", "w")

    prev_time = start_time = time_ns()
    end_time = start_time + 10_000_000_000
    try
        while time_ns() < end_time
            now_time = time_ns()

            n = ADFMCOMMS2.recv!(adapter, recv_buffer)
            total_bytes += n * sizeof(eltype(recv_buffer))
            write(io, @view recv_buffer[1:n])
            
            if now_time - prev_time >= 1_000_000_000
                elapsed_s = (now_time - start_time) / 1_000_000_000
                rate = total_bytes / elapsed_s
                rate_val, rate_unit = format_si(rate)
                total_val, total_unit = format_si(total_bytes)
                print("rate: ", @sprintf("%.3f", rate_val), " ", rate_unit, "B/s, total: ",
                      @sprintf("%.3f", total_val), " ", total_unit, "B\r")
                prev_time = now_time
            end
            yield()
        end
    catch e
        if e isa InterruptException
            @warn "Interrupted by Ctrl-C"
        else
            rethrow()
        end
    finally
        close(io)
    end

    ADFMCOMMS2.close!(adapter)
    
end
