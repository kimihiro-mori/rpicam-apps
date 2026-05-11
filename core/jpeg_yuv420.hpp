#pragma once

#include <cstddef>
#include <cstdint>

#include "core/stream_info.hpp"

struct jpeg_compress_struct;
struct jpeg_error_mgr;

class JpegYuv420Encoder
{
public:
    JpegYuv420Encoder();
    ~JpegYuv420Encoder();

    bool Encode(const uint8_t *yuv420, const StreamInfo &info, int quality,
                uint8_t *&out_buf, size_t &out_len);

private:
    jpeg_compress_struct *cinfo_;
    jpeg_error_mgr *jerr_;
};
