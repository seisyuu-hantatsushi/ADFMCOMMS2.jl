
using Libdl

const libAD9361 = dlpath(dlopen("libad9361", true))

export
    ad9361_set_bb_rate

function ad9361_set_bb_rate(dev::Ptr{iio_device}, samplingRate::UInt32)
    return ccall((:ad9361_set_bb_rate, libAD9361),
                 Cint, (Ptr{iio_device}, Cuint),
                 dev, samplingRate)
end
