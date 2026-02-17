/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_vid.cpp - libcamera video record app.
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <fcntl.h>

#include "core/rpicam_encoder.hpp"
#include "output/output.hpp"
#include "core/jpeg_yuv420.hpp"

using namespace std::placeholders;

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
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
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options->signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if ((signal_received == SIGUSR2) || (signal_received == SIGPIPE))
			key = 'x';
		signal_received = 0;
	}
	return key;
}

static int get_colourspace_flags(std::string const &codec)
{
	if (codec == "mjpeg" || codec == "yuv420")
		return RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
	else
		return RPiCamEncoder::FLAG_VIDEO_NONE;
}

struct UdpJpegSender
{
	int fd = -1;
	sockaddr_in addr {};
	bool enabled = false;

	~UdpJpegSender()
	{
		if (fd >= 0) ::close(fd);
	}

	static std::optional<UdpJpegSender> FromEnv()
	{
		// Check for an environment variable of the "RPICAM_LORES_UDP"
		const char *s = std::getenv("RPICAM_LORES_UDP");
		if (!s || !*s) return std::nullopt;

		// Expect it to be in the form "IP:PORT", e.g. "127.0.0.1:5001"
		std::string spec(s);
		auto pos = spec.find(':');
		if (pos == std::string::npos) return std::nullopt;

		std::string ip = spec.substr(0, pos);
		int port = std::atoi(spec.substr(pos + 1).c_str());
		if (port <= 0) return std::nullopt;

		// Create a UDP socket for sending JPEG frames to the specified address
		UdpJpegSender sender;
		sender.fd = ::socket(AF_INET, SOCK_DGRAM, 0);
		if (sender.fd < 0) return std::nullopt;

		std::memset(&sender.addr, 0, sizeof(sender.addr));
		sender.addr.sin_family = AF_INET;
		sender.addr.sin_port = htons(port);
		if (::inet_pton(AF_INET, ip.c_str(), &sender.addr.sin_addr) != 1) return std::nullopt;

		// Set the socket to non-blocking mode
		int flags = fcntl(sender.fd, F_GETFL, 0);
		fcntl(sender.fd, F_SETFL, flags | O_NONBLOCK);

		sender.enabled = true;
		return sender;
	}

	void SendOneFrame(const uint8_t *data, size_t len)
	{
		if (!enabled) return;
		if (len > 65000)
		{
			LOG_ERROR("UDP JPEG drop: packet too large (" << len << " bytes)");
			return;
		}
		::sendto(fd, data, len, 0, (sockaddr *)&addr, sizeof(addr));
	}
};

// The main even loop for the application.

static void event_loop(RPiCamEncoder &app)
{
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->codec));

	// Set up the lores stream and UDP sender
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	StreamInfo lores_info;
	libcamera::Stream *lores_stream = app.LoresStream(&lores_info);

	auto senderOpt = UdpJpegSender::FromEnv();
	UdpJpegSender sender;
	if (senderOpt) sender = std::move(*senderOpt);

	int interval_ms = 33; // default ~30fps
	if (const char *s = std::getenv("RPICAM_LORES_INTERVAL_MS"))
		interval_ms = std::max(1, std::atoi(s));

	int jpeg_quality = 70;
	if (const char *s = std::getenv("RPICAM_LORES_JPEG_QUALITY"))
		jpeg_quality = std::min(95, std::max(30, std::atoi(s)));

	auto last_sent = std::chrono::high_resolution_clock::now();

	JpegYuv420Encoder lores_jpeg_enc;
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	signal(SIGINT, default_signal_handler);
	// SIGPIPE gets raised when trying to write to an already closed socket. This can happen, when
	// you're using TCP to stream to VLC and the user presses the stop button in VLC. Catching the
	// signal to be able to react on it, otherwise the app terminates.
	signal(SIGPIPE, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

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
			return;
		else if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		int key = get_key_or_signal(options, p);
		if (key == '\n')
			output->Signal();

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->frames && options->timeout &&
					   ((now - start_time) > options->timeout.value);
		bool frameout = options->frames && count >= options->frames;
		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				LOG(1, "Halting: reached timeout of " << options->timeout.get<std::chrono::milliseconds>()
													  << " milliseconds.");
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (!app.EncodeBuffer(completed_request, app.VideoStream()))
		{
			// Keep advancing our "start time" if we're still waiting to start recording (e.g.
			// waiting for synchronisation with another camera).
			start_time = now;
			count = 0; // reset the "frames encoded" counter too
		}

		// Send the lores stream as JPEG over UDP if enabled
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		if (lores_stream && sender.enabled)
		{
			auto now2 = std::chrono::high_resolution_clock::now();
			if (std::chrono::duration_cast<std::chrono::milliseconds>(now2 - last_sent).count() >= interval_ms)
			{
				last_sent = now2;
				FrameBuffer *buf = completed_request->buffers[lores_stream];
				if (buf)
				{
					BufferReadSync r(&app, buf);
					libcamera::Span<uint8_t> span = r.Get()[0];

					uint8_t *jpg = nullptr;
					size_t jpg_len = 0;

					if (lores_jpeg_enc.Encode(span.data(), lores_info, jpeg_quality, jpg, jpg_len))
					{
						sender.SendOneFrame(jpg, jpg_len);
						free(jpg);
					}
				}
			}
		}
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
