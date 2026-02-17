#include "core/jpeg_yuv420.hpp"

#include <algorithm>
#include <jpeglib.h>
#include <cstdlib>

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
using jpeg_mem_len_t = size_t;
#else
using jpeg_mem_len_t = unsigned long;
#endif

JpegYuv420Encoder::JpegYuv420Encoder()
{
    cinfo_ = new jpeg_compress_struct();
    jerr_  = new jpeg_error_mgr();

    cinfo_->err = jpeg_std_error(jerr_);
    jpeg_create_compress(cinfo_);
}

JpegYuv420Encoder::~JpegYuv420Encoder()
{
    if (cinfo_) {
        jpeg_destroy_compress(cinfo_);
        delete cinfo_;
        cinfo_ = nullptr;
    }
    delete jerr_;
    jerr_ = nullptr;
}

bool JpegYuv420Encoder::Encode(const uint8_t *yuv420, const StreamInfo &info, int quality,
                               uint8_t *&out_buf, size_t &out_len)
{
    out_buf = nullptr;
    out_len = 0;

    if (!yuv420) return false;
    if ((info.width & 1) || (info.height & 1)) return false;
    if (info.stride == 0) return false;

    jpeg_abort_compress(cinfo_);

    quality = std::max(1, std::min(quality, 100));

    cinfo_->image_width = info.width;
    cinfo_->image_height = info.height;
    cinfo_->input_components = 3;
    cinfo_->in_color_space = JCS_YCbCr;
    cinfo_->restart_interval = 0;

    jpeg_set_defaults(cinfo_);
    cinfo_->raw_data_in = TRUE;
    jpeg_set_quality(cinfo_, quality, TRUE);

    jpeg_mem_len_t jpeg_len = 0;
    jpeg_mem_dest(cinfo_, &out_buf, &jpeg_len);

    jpeg_start_compress(cinfo_, TRUE);

    int stride2 = info.stride / 2;
    const uint8_t *Y = yuv420;
    const uint8_t *U = Y + info.stride * info.height;
    const uint8_t *V = U + stride2 * (info.height / 2);

    const uint8_t *Y_max = U - info.stride;
    const uint8_t *U_max = V - stride2;
    const uint8_t *V_max = U_max + stride2 * (info.height / 2);

    JSAMPROW y_rows[16];
    JSAMPROW u_rows[8];
    JSAMPROW v_rows[8];

    for (const uint8_t *Y_row = Y, *U_row = U, *V_row = V; cinfo_->next_scanline < info.height;)
    {
        for (int i = 0; i < 16; i++, Y_row += info.stride)
            y_rows[i] = (JSAMPROW)std::min(Y_row, Y_max);

        for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2) {
            u_rows[i] = (JSAMPROW)std::min(U_row, U_max);
            v_rows[i] = (JSAMPROW)std::min(V_row, V_max);
        }

        JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
        jpeg_write_raw_data(cinfo_, rows, 16);
    }

    jpeg_finish_compress(cinfo_);

    out_len = (size_t)jpeg_len;
    return out_buf && out_len > 0;
}
