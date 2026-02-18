/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_encoder.cpp - libcamera video encoding class.
 */

#pragma once

#include "core/rpicam_app.hpp"
#include "core/stream_info.hpp"
#include "core/video_options.hpp"

#include "encoder/encoder.hpp"

typedef std::function<void(void *, size_t, int64_t, bool)> EncodeOutputReadyCallback;
typedef std::function<void(libcamera::ControlList &)> MetadataReadyCallback;

class RPiCamEncoder : public RPiCamApp
{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;

	RPiCamEncoder() : RPiCamApp(std::make_unique<VideoOptions>()) {}

	void StartEncoder()
	{
		createEncoder();
		encoder_->SetInputDoneCallback(std::bind(&RPiCamEncoder::encodeBufferDone, this, std::placeholders::_1));
		encoder_->SetOutputReadyCallback(encode_output_ready_callback_);
	}
	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { encode_output_ready_callback_ = callback; }
	void SetMetadataReadyCallback(MetadataReadyCallback callback) { metadata_ready_callback_ = callback; }
	void EncodeBuffer(CompletedRequestPtr &completed_request, Stream *stream)
	{
		assert(encoder_);
		StreamInfo info = GetStreamInfo(stream);
		FrameBuffer *buffer = completed_request->buffers[stream];
		BufferReadSync r(this, buffer);
		libcamera::Span span = r.Get()[0];
		void *mem = span.data();
		if (!buffer || !mem)
			throw std::runtime_error("no buffer to encode");
		auto ts = completed_request->metadata.get(controls::SensorTimestamp);
		int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			encode_buffer_queue_.push(completed_request); // creates a new reference
		}
		encoder_->EncodeBuffer(buffer->planes()[0].fd.get(), span.size(), mem, info, timestamp_ns / 1000);
	}
	VideoOptions *GetOptions() const { return static_cast<VideoOptions *>(options_.get()); }
	void StopEncoder() { encoder_.reset(); }

	// Return the number of frames currently queued for encoding (i.e. buffers
	// held by the encoder pipeline that haven't been returned yet).
	size_t EncodeBufferQueueSize() const
	{
		std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
		return encode_buffer_queue_.size();
	}

protected:
	virtual void createEncoder()
	{
		StreamInfo info;
		VideoStream(&info);
		if (!info.width || !info.height || !info.stride)
			throw std::runtime_error("video steam is not configured");
		encoder_ = std::unique_ptr<Encoder>(Encoder::Create(GetOptions(), info));
	}
	std::unique_ptr<Encoder> encoder_;

private:
	void encodeBufferDone(void *mem)
	{
		// If non-NULL, mem would indicate which buffer has been completed, but
		// currently we're just assuming everything is done in order. (We could
		// handle this by replacing the queue with a vector of <mem, completed_request>
		// pairs.)
		assert(mem == nullptr);
		{
			std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
			if (encode_buffer_queue_.empty())
				throw std::runtime_error("no buffer available to return");
			CompletedRequestPtr &completed_request = encode_buffer_queue_.front();
			if (metadata_ready_callback_ && !GetOptions()->metadata.empty())
				metadata_ready_callback_(completed_request->metadata);
			encode_buffer_queue_.pop(); // drop shared_ptr reference
		}
	}

	std::queue<CompletedRequestPtr> encode_buffer_queue_;
	mutable std::mutex encode_buffer_queue_mutex_;
	EncodeOutputReadyCallback encode_output_ready_callback_;
	MetadataReadyCallback metadata_ready_callback_;
};
