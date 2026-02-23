/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_vid.cpp - libcamera video record app.
 */

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <mutex>
#include <poll.h>
#include <signal.h>
#include <thread>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "core/rpicam_encoder.hpp"
#include "core/jpeg_yuv420.hpp"
#include "output/output.hpp"

using namespace std::placeholders;
using Clock = std::chrono::high_resolution_clock;

// ---------------------------------------------------------------------------
// Signal / keypress helpers
// ---------------------------------------------------------------------------

static int signal_received;

static void default_signal_handler(int sig)
{
	signal_received = sig;
	LOG(1, "Received signal " << sig);
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
	int key = 0;
	if (signal_received == SIGINT)
		return 'x';
	if (options->keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *s = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&s, &len, stdin);
			key = s[0];
		}
	}
	if (options->signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if (signal_received == SIGUSR2 || signal_received == SIGPIPE)
			key = 'x';
		signal_received = 0;
	}
	return key;
}

static int get_colourspace_flags(std::string const &codec)
{
	return (codec == "mjpeg" || codec == "yuv420")
		? RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE
		: RPiCamEncoder::FLAG_VIDEO_NONE;
}

// ---------------------------------------------------------------------------
// UDP JPEG sender – reads RPICAM_LORES_UDP env ("ip:port")
// ---------------------------------------------------------------------------

class UdpJpegSender
{
public:
	UdpJpegSender() = default;
	~UdpJpegSender() { if (fd_ >= 0) ::close(fd_); }

	UdpJpegSender(const UdpJpegSender &) = delete;
	UdpJpegSender &operator=(const UdpJpegSender &) = delete;
	UdpJpegSender(UdpJpegSender &&o) noexcept { steal(o); }
	UdpJpegSender &operator=(UdpJpegSender &&o) noexcept
	{
		if (this != &o) { if (fd_ >= 0) ::close(fd_); steal(o); }
		return *this;
	}

	bool enabled() const { return fd_ >= 0; }

	static UdpJpegSender Create()
	{
		const char *s = std::getenv("RPICAM_LORES_UDP");
		if (!s || !*s) return {};
		std::string spec(s);
		auto pos = spec.find(':');
		if (pos == std::string::npos) return {};

		std::string ip = spec.substr(0, pos);
		int port = std::atoi(spec.substr(pos + 1).c_str());
		if (port <= 0) return {};

		UdpJpegSender out;
		out.fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
		if (out.fd_ < 0) return {};

		std::memset(&out.addr_, 0, sizeof(out.addr_));
		out.addr_.sin_family = AF_INET;
		out.addr_.sin_port = htons(port);
		if (::inet_pton(AF_INET, ip.c_str(), &out.addr_.sin_addr) != 1) return {};

		int fl = fcntl(out.fd_, F_GETFL, 0);
		fcntl(out.fd_, F_SETFL, fl | O_NONBLOCK);
		return out;
	}

	void Send(const uint8_t *data, size_t len)
	{
		if (len > 65000) { LOG_ERROR("UDP JPEG drop: " << len << " bytes"); return; }
		::sendto(fd_, data, len, 0, (sockaddr *)&addr_, sizeof(addr_));
	}

private:
	int fd_ = -1;
	sockaddr_in addr_{};

	void steal(UdpJpegSender &o)
	{
		fd_ = o.fd_; addr_ = o.addr_;
		o.fd_ = -1;
	}
};

// ---------------------------------------------------------------------------
// Lores stream → JPEG → UDP forwarding (fully async)
//
// The main thread only copies raw YUV pixels into a staging buffer under a
// short lock.  A dedicated worker thread does the JPEG encode + UDP send,
// keeping the main event loop as lean as possible.
// ---------------------------------------------------------------------------

class LoresForwarder
{
public:
	explicit LoresForwarder(UdpJpegSender &sender)
		: sender_(sender)
	{
		if (const char *s = std::getenv("RPICAM_LORES_INTERVAL_MS"))
			interval_ms_ = std::max(1, std::atoi(s));
		if (const char *s = std::getenv("RPICAM_LORES_JPEG_QUALITY"))
			quality_ = std::clamp(std::atoi(s), 30, 95);
	}

	bool active() const { return sender_.enabled(); }

	void Start()
	{
		if (!active()) return;
		thread_ = std::thread([this] { workerLoop(); });
	}

	void Stop()
	{
		if (!active()) return;
		{ std::lock_guard<std::mutex> lk(mtx_); stop_ = true; }
		cv_.notify_one();
		if (thread_.joinable()) thread_.join();
	}

	/* Called from event loop — copies raw YUV pixels under a short lock.
	   JPEG encoding happens on the worker thread. */
	void Enqueue(RPiCamEncoder &app, CompletedRequestPtr &req,
				 libcamera::Stream *stream, const StreamInfo &info)
	{
		if (!active() || !stream) return;

		auto now = Clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_enqueue_).count() < interval_ms_)
			return;
		last_enqueue_ = now;

		libcamera::FrameBuffer *fb = req->buffers[stream];
		if (!fb) return;

		BufferReadSync r(&app, fb);
		libcamera::Span<uint8_t> span = r.Get()[0];
		size_t nbytes = info.stride * info.height * 3 / 2; // YUV420
		if (span.size() < nbytes) nbytes = span.size();

		{
			std::lock_guard<std::mutex> lk(mtx_);
			if (yuv_buf_.size() < nbytes)
				yuv_buf_.resize(nbytes);
			std::memcpy(yuv_buf_.data(), span.data(), nbytes);
			yuv_info_ = info;
			yuv_ready_ = true;
		}
		cv_.notify_one();
	}

private:
	void workerLoop()
	{
		JpegYuv420Encoder enc;

		while (true)
		{
			std::vector<uint8_t> local_yuv;
			StreamInfo local_info;

			{
				std::unique_lock<std::mutex> lk(mtx_);
				cv_.wait_for(lk, std::chrono::milliseconds(interval_ms_),
							 [this] { return stop_ || yuv_ready_; });
				if (stop_) break;
				if (!yuv_ready_) continue;
				local_yuv.swap(yuv_buf_);
				local_info = yuv_info_;
				yuv_ready_ = false;
			}

			uint8_t *jpg = nullptr;
			size_t jpg_len = 0;
			if (enc.Encode(local_yuv.data(), local_info, quality_, jpg, jpg_len))
			{
				sender_.Send(jpg, jpg_len);
				free(jpg);
			}

			// Return buffer for reuse
			{
				std::lock_guard<std::mutex> lk(mtx_);
				if (yuv_buf_.empty())
					yuv_buf_.swap(local_yuv);
			}
		}
	}

	UdpJpegSender &sender_;
	int interval_ms_ = 33;
	int quality_ = 70;
	Clock::time_point last_enqueue_ = Clock::now();

	std::mutex mtx_;
	std::condition_variable cv_;
	std::vector<uint8_t> yuv_buf_;
	StreamInfo yuv_info_;
	bool yuv_ready_ = false;
	bool stop_ = false;
	std::thread thread_;
};

// ---------------------------------------------------------------------------
// Main event loop
// ---------------------------------------------------------------------------

static void event_loop(RPiCamEncoder &app)
{
	VideoOptions const *options = app.GetOptions();

	auto output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->codec));

	// Lores UDP forwarding setup
	StreamInfo lores_info;
	libcamera::Stream *lores_stream = app.LoresStream(&lores_info);
	UdpJpegSender sender = UdpJpegSender::Create();
	LoresForwarder lores(sender);
	lores.Start();

	app.StartEncoder();
	app.StartCamera();
	auto start_time = Clock::now();

	for (int sig : {SIGUSR1, SIGUSR2, SIGINT, SIGPIPE})
		signal(sig, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	auto shutdown = [&] { lores.Stop(); };

	// Frame-drop monitoring: log a warning every reporting interval if the
	// measured framerate falls below the requested rate.
	const double target_fps = options->framerate.value_or(30);
	const auto report_interval = std::chrono::seconds(5);
	auto last_report = Clock::now();
	unsigned int frames_since_report = 0;
	uint64_t last_sensor_ts = 0;
	unsigned int drop_count = 0;
	size_t enc_queue_peak = 0;

	for (unsigned int count = 0; ; count++)
	{
		RPiCamEncoder::Msg msg = app.Wait();

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamEncoder::MsgType::Quit)
		{
			shutdown();
			return;
		}
		if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		int key = get_key_or_signal(options, p);
		if (key == '\n')
			output->Signal();

		LOG(2, "Viewfinder frame " << count);
		auto now = Clock::now();
		bool timeout = !options->frames && options->timeout &&
					   ((now - start_time) > options->timeout.value);
		bool frameout = options->frames && count >= options->frames;

		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				LOG(1, "Halting: reached timeout of "
					<< options->timeout.get<std::chrono::milliseconds>() << " milliseconds.");
			app.StopCamera();
			app.StopEncoder();
			shutdown();
			LOG(1, "Total frames: " << count << ", detected drops: " << drop_count
				<< ", enc queue at exit: " << app.EncodeBufferQueueSize());
			return;
		}

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

		// ---- drop detection via sensor timestamp gaps ----
		auto ts_opt = completed_request->metadata.get(libcamera::controls::SensorTimestamp);
		if (ts_opt)
		{
			uint64_t ts = *ts_opt;
			if (last_sensor_ts && target_fps > 0)
			{
				double gap_us = (ts - last_sensor_ts) / 1000.0;
				double expected_us = 1e6 / target_fps;
				// If the gap is more than 1.5× the expected interval, count a drop
				if (gap_us > expected_us * 1.5)
				{
					unsigned int missed = static_cast<unsigned int>(gap_us / expected_us + 0.5) - 1;
					drop_count += missed;
					LOG(1, "Frame drop detected: gap " << static_cast<int>(gap_us)
						<< "us (expected " << static_cast<int>(expected_us) << "us), ~"
						<< missed << " frame(s) missed");
				}
			}
			last_sensor_ts = ts;
		}

		// ---- periodic throughput report ----
		frames_since_report++;
		size_t enc_q = app.EncodeBufferQueueSize();
		if (enc_q > enc_queue_peak)
			enc_queue_peak = enc_q;
		if (now - last_report >= report_interval)
		{
			double elapsed_s = std::chrono::duration<double>(now - last_report).count();
			double measured_fps = frames_since_report / elapsed_s;
			LOG(1, "Throughput: " << std::fixed << std::setprecision(1) << measured_fps
				<< " fps (target " << target_fps
				<< "), enc queue: " << enc_q << "/" << options->buffer_count
				<< " (peak " << enc_queue_peak << ")"
				<< ", drops: " << drop_count);
			frames_since_report = 0;
			last_report = now;
			enc_queue_peak = 0;
		}

		app.EncodeBuffer(completed_request, app.VideoStream());
		lores.Enqueue(app, completed_request, lores_stream, lores_info);
		if (!options->nopreview)
			app.ShowPreview(completed_request, app.VideoStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamEncoder app;
		VideoOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();
			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
