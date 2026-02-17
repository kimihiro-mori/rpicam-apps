/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>

#include <jpeglib.h>

#include "mjpeg_encoder.hpp"
#include "core/jpeg_yuv420.hpp"

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif

MjpegEncoder::MjpegEncoder(VideoOptions const *options)
	: Encoder(options), abortEncode_(false), abortOutput_(false), index_(0)
{
	output_thread_ = std::thread(&MjpegEncoder::outputThread, this);
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i] = std::thread(std::bind(&MjpegEncoder::encodeThread, this, i));
	LOG(2, "Opened MjpegEncoder");
}

MjpegEncoder::~MjpegEncoder()
{
	abortEncode_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i].join();
	abortOutput_ = true;
	output_thread_.join();
	LOG(2, "MjpegEncoder closed");
}

void MjpegEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
{
	std::lock_guard<std::mutex> lock(encode_mutex_);
	EncodeItem item = { mem, info, timestamp_us, index_++ };
	encode_queue_.push(item);
	encode_cond_var_.notify_all();
}



void MjpegEncoder::encodeThread(int num)
{
    JpegYuv420Encoder enc;
    std::chrono::duration<double> encode_time(0);
    uint32_t frames = 0;

    EncodeItem encode_item;
    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(encode_mutex_);
            while (true)
            {
                using namespace std::chrono_literals;
                if (abortEncode_ && encode_queue_.empty())
                {
                    if (frames)
                        LOG(2, "Encode " << frames
                                         << " frames, average time " << encode_time.count() * 1000 / frames
                                         << "ms");
                    return;
                }
                if (!encode_queue_.empty())
                {
                    encode_item = encode_queue_.front();
                    encode_queue_.pop();
                    break;
                }
                encode_cond_var_.wait_for(lock, 200ms);
            }
        }

        uint8_t *encoded_buffer = nullptr;
        size_t buffer_len = 0;

        auto start_time = std::chrono::high_resolution_clock::now();
        bool ok = enc.Encode(static_cast<const uint8_t *>(encode_item.mem),
                             encode_item.info,
                             options_->Get().quality,
                             encoded_buffer,
                             buffer_len);
        encode_time += (std::chrono::high_resolution_clock::now() - start_time);
        frames++;

        if (!ok || !encoded_buffer || buffer_len == 0)
        {
            if (encoded_buffer)
                free(encoded_buffer);

            OutputItem output_item = { nullptr, 0, encode_item.timestamp_us, encode_item.index };
            std::lock_guard<std::mutex> lock(output_mutex_);
            output_queue_[num].push(output_item);
            output_cond_var_.notify_one();
            continue;
        }

        OutputItem output_item = { encoded_buffer, buffer_len, encode_item.timestamp_us, encode_item.index };
        std::lock_guard<std::mutex> lock(output_mutex_);
        output_queue_[num].push(output_item);
        output_cond_var_.notify_one();
    }
}

void MjpegEncoder::outputThread()
{
	OutputItem item;
	uint64_t index = 0;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(output_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				// We look for the thread that's completed the frame we want next.
				// If we don't find it, we wait.
				//
				// Must also check for an abort signal, and if set, all queues must
				// be empty. This is done first to ensure all frame callbacks have
				// had a chance to run.
				bool abort = abortOutput_ ? true : false;
				for (auto &q : output_queue_)
				{
					if (abort && !q.empty())
						abort = false;

					if (!q.empty() && q.front().index == index)
					{
						item = q.front();
						q.pop();
						goto got_item;
					}
				}
				if (abort)
					return;

				output_cond_var_.wait_for(lock, 200ms);
			}
		}
	got_item:
		input_done_callback_(nullptr);

		output_ready_callback_(item.mem, item.bytes_used, item.timestamp_us, true);
		free(item.mem);
		index++;
	}
}
