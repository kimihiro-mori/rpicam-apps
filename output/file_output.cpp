/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "file_output.hpp"

// One record per saved frame in the <output>.frames sidecar (native little-endian).
struct FrameSidecarRecord
{
	uint32_t index;
	uint32_t byte_size;
	int64_t sensor_ts_ns;
	uint64_t byte_offset;
};
static_assert(sizeof(FrameSidecarRecord) == 24, "sidecar record must stay 24 bytes");

FileOutput::FileOutput(VideoOptions const *options)
	: Output(options), fp_(nullptr), count_(0), file_start_time_ms_(0)
{
	const char *env = std::getenv("RPICAM_FRAMES_SIDECAR");
	frames_sidecar_enabled_ = env && std::strcmp(env, "1") == 0;
}

FileOutput::~FileOutput()
{
	closeFile();
}

void FileOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	// We need to open a new file if we're in "segment" mode and our segment is full
	// (though we have to wait for the next I frame), or if we're in "split" mode
	// and recording is being restarted (this is necessarily an I-frame already).
	if (fp_ == nullptr ||
		(options_->segment && (flags & FLAG_KEYFRAME) &&
		 timestamp_us / 1000 - file_start_time_ms_ > options_->segment) ||
		(options_->split && (flags & FLAG_RESTART)))
	{
		closeFile();
		openFile(timestamp_us);
	}

	LOG(2, "FileOutput: output buffer " << mem << " size " << size);
	if (fp_ && size)
	{
		if (fwrite(mem, size, 1, fp_) != 1)
			throw std::runtime_error("failed to write output bytes");
		if (options_->flush)
			fflush(fp_);

		if (fp_frames_)
		{
			FrameSidecarRecord rec;
			rec.index = frame_index_;
			rec.byte_size = static_cast<uint32_t>(size);
			rec.sensor_ts_ns = raw_sensor_ts_us_ * 1000;
			rec.byte_offset = bytes_written_;
			if (fwrite(&rec, sizeof(rec), 1, fp_frames_) != 1)
			{
				LOG_ERROR("FileOutput: frames sidecar write failed, disabling");
				fclose(fp_frames_);
				fp_frames_ = nullptr;
			}
			else if (options_->flush)
				fflush(fp_frames_);
		}
		bytes_written_ += size;
		frame_index_++;
	}
}

void FileOutput::openFile(int64_t timestamp_us)
{
	if (options_->output == "-")
		fp_ = stdout;
	else if (!options_->output.empty())
	{
		// Generate the next output file name.
		char filename[256];
		int n;

		if (options_->output.find("%s") != std::string::npos)
		{
			// Timestamp-based naming: %s is replaced with YYYYMMDD_HHMMSS
			auto now = std::chrono::system_clock::now();
			auto tt = std::chrono::system_clock::to_time_t(now);
			struct tm tm;
			localtime_r(&tt, &tm);
			char ts[20];
			strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
			n = snprintf(filename, sizeof(filename), options_->output.c_str(), ts);
		}
		else
		{
			n = snprintf(filename, sizeof(filename), options_->output.c_str(), count_);
		}
		count_++;
		if (options_->wrap)
			count_ = count_ % options_->wrap;
		if (n < 0)
			throw std::runtime_error("failed to generate filename");

		fp_ = fopen(filename, "w");
		if (!fp_)
			throw std::runtime_error("failed to open output file " + std::string(filename));
		LOG(2, "FileOutput: opened output file " << filename);

		file_start_time_ms_ = timestamp_us / 1000;

		frame_index_ = 0;
		bytes_written_ = 0;
		if (frames_sidecar_enabled_)
		{
			std::string sidecar = std::string(filename) + ".frames";
			fp_frames_ = fopen(sidecar.c_str(), "w");
			if (!fp_frames_)
				LOG_ERROR("FileOutput: failed to open frames sidecar " << sidecar);
		}
	}
}

void FileOutput::closeFile()
{
	if (fp_)
	{
		if (options_->flush)
			fflush(fp_);
		if (fp_ != stdout)
			fclose(fp_);
		fp_ = nullptr;
	}
	if (fp_frames_)
	{
		fclose(fp_frames_);
		fp_frames_ = nullptr;
	}
}
