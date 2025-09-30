/*****************************************************************************
#                                                                            #
#    uStreamer - Lightweight and fast MJPEG-HTTP streamer.                   #
#                                                                            #
#    Copyright (C) 2018-2024  Maxim Devaev <mdevaev@gmail.com>               #
#                                                                            #
#    This program is free software: you can redistribute it and/or modify    #
#    it under the terms of the GNU General Public License as published by    #
#    the Free Software Foundation, either version 3 of the License, or       #
#    (at your option) any later version.                                     #
#                                                                            #
#    This program is distributed in the hope that it will be useful,         #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of          #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           #
#    GNU General Public License for more details.                            #
#                                                                            #
#    You should have received a copy of the GNU General Public License       #
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.  #
#                                                                            #
*****************************************************************************/


#include "drm.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/mman.h>
#include <sys/stat.h>
#ifdef __linux__
#	include <sys/sysmacros.h>
#endif

#include <linux/videodev2.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <libdrm/drm.h>
#include <libyuv.h>

#include "../types.h"
#include "../errors.h"
#include "../tools.h"
#include "../logging.h"
#include "../frame.h"
#include "../frametext.h"
#include "../capture.h"
#include "../unjpeg.h"


static void _drm_vsync_callback(int fd, uint n_frame, uint sec, uint usec, void *v_buf);
static int _drm_check_status(us_drm_s *drm);
static void _drm_ensure_dpms_power(us_drm_s *drm, bool on);
static int _drm_init_buffers(us_drm_s *drm, const us_capture_s *cap);
static int _drm_find_sink(us_drm_s *drm, uint width, uint height, float hz);
static void _drm_calculate_center(us_drm_center_s *center, uint src_w, uint src_h, uint dst_w, uint dst_h);
static void _drm_convert_yuyv_simple(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h);
static void _drm_convert_rgb24(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h);
static void _drm_convert_bgr24(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h);
static void _drm_convert_mjpeg(const u8 *src_data, size_t src_size, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h);

// Platform-specific implementation functions
static int _drm_expose_rpi_v4p_impl(us_drm_s *drm, const us_capture_hwbuf_s *hw);
static int _drm_expose_amlogic_display_impl(us_drm_s *drm, const us_capture_hwbuf_s *hw);

static drmModeModeInfo *_find_best_mode(drmModeConnector *conn, uint width, uint height, float hz);
static u32 _find_dpms(int fd, drmModeConnector *conn);
static u32 _find_crtc(int fd, drmModeRes *res, drmModeConnector *conn, u32 *taken_crtcs);
static const char *_connector_type_to_string(u32 type);
static float _get_refresh_rate(const drmModeModeInfo *mode);
static us_drm_platform_e _detect_drm_platform(int fd);


#define _LOG_ERROR(x_msg, ...)	US_LOG_ERROR("DRM: " x_msg, ##__VA_ARGS__)
#define _LOG_PERROR(x_msg, ...)	US_LOG_PERROR("DRM: " x_msg, ##__VA_ARGS__)
#define _LOG_INFO(x_msg, ...)		US_LOG_INFO("DRM: " x_msg, ##__VA_ARGS__)
#define _LOG_VERBOSE(x_msg, ...)	US_LOG_VERBOSE("DRM: " x_msg, ##__VA_ARGS__)
#define _LOG_DEBUG(x_msg, ...)	US_LOG_DEBUG("DRM: " x_msg, ##__VA_ARGS__)


us_drm_s *us_drm_init(void) {
	us_drm_runtime_s *run;
	US_CALLOC(run, 1);
	run->fd = -1;
	run->status_fd = -1;
	run->dpms_state = -1;
	run->opened = -1;
	run->has_vsync = true;
	run->exposing_dma_fd = -1;
	run->ft = us_frametext_init();
	run->detected_bpp = 24;  // 默认24位
	run->platform = US_DRM_PLATFORM_UNKNOWN;  // Will be detected during open

	us_drm_s *drm;
	US_CALLOC(drm, 1);
	// drm->path = "/dev/dri/card0";
	drm->path = strdup("/dev/dri/by-path/platform-gpu-card");
	drm->port = NULL; // Auto-detect by default, can be overridden via --drm-port
	drm->timeout = 5;
	drm->blank_after = 5;
	drm->center_mode = false;
	drm->run = run;
	return drm;
}

void us_drm_destroy(us_drm_s *drm) {
	us_frametext_destroy(drm->run->ft);
	US_DELETE(drm->run, free);
	US_DELETE(drm->path, free);
	US_DELETE(drm->port, free);
	US_DELETE(drm, free); // cppcheck-suppress uselessAssignmentPtrArg
}

int us_drm_open(us_drm_s *drm, const us_capture_s *cap) {
	us_drm_runtime_s *const run = drm->run;

	assert(run->fd < 0);

	switch (_drm_check_status(drm)) {
		case 0: break;
		case US_ERROR_NO_DEVICE: goto unplugged;
		default: goto error;
	}

	if (drm->port != NULL) {
		_LOG_INFO("Using passthrough: %s[%s]", drm->path, drm->port);
	} else {
		_LOG_INFO("Using passthrough: %s[auto-detect]", drm->path);
	}
	_LOG_INFO("Configuring DRM device for %s ...", (cap == NULL ? "STUB" : "DMA"));

	if ((run->fd = open(drm->path, O_RDWR | O_CLOEXEC | O_NONBLOCK)) < 0) {
		_LOG_PERROR("Can't open DRM device");
		goto error;
	}
	_LOG_DEBUG("DRM device fd=%d opened", run->fd);

	// Improve DRM master control management
	_LOG_DEBUG("Checking current DRM master status...");
	int master_status = drmIsMaster(run->fd);
	_LOG_DEBUG("Current DRM master status: %s", master_status ? "master" : "not master");

	// Drop master first, then acquire it to ensure clean state
	drmDropMaster(run->fd);
	if (drmSetMaster(run->fd) < 0) {
		_LOG_ERROR("Can't acquire DRM master control: %s", strerror(errno));
		_LOG_INFO("Hint: Make sure no other programs are using the display (close X11/Wayland sessions)");
		_LOG_INFO("Or try switching to a virtual terminal (Ctrl+Alt+F1-F6)");
		goto error;
	}
	_LOG_DEBUG("DRM master control acquired successfully");

	// Detect platform type for appropriate handling
	run->platform = _detect_drm_platform(run->fd);
	const char *platform_name = "Unknown";
	switch (run->platform) {
		case US_DRM_PLATFORM_RPI: platform_name = "Raspberry Pi"; break;
		case US_DRM_PLATFORM_AMLOGIC: platform_name = "Amlogic"; break;
		case US_DRM_PLATFORM_GENERIC: platform_name = "Generic"; break;
		default: break;
	}
	_LOG_INFO("Detected DRM platform: %s", platform_name);

	int stub = 0; // Open the real device with DMA
	if (cap == NULL) {
		stub = US_DRM_STUB_USER;
	} else if (cap->run->format != V4L2_PIX_FMT_RGB24 && cap->run->format != V4L2_PIX_FMT_BGR24 && 
	           cap->run->format != V4L2_PIX_FMT_YUYV && cap->run->format != V4L2_PIX_FMT_MJPEG) {
		stub = US_DRM_STUB_BAD_FORMAT;
		char fourcc_str[8];
		us_fourcc_to_string(cap->run->format, fourcc_str, 8);
		_LOG_ERROR("Input format %s is not supported, forcing to STUB ...", fourcc_str);
	}

#	define CHECK_CAP(x_cap) { \
			_LOG_DEBUG("Checking %s ...", #x_cap); \
			u64 m_check; \
			if (drmGetCap(run->fd, x_cap, &m_check) < 0) { \
				_LOG_PERROR("Can't check " #x_cap); \
				goto error; \
			} \
			if (!m_check) { \
				_LOG_ERROR(#x_cap " is not supported"); \
				goto error; \
			} \
		}
	CHECK_CAP(DRM_CAP_DUMB_BUFFER);
	if (stub == 0) {
		CHECK_CAP(DRM_CAP_PRIME);
	}
#	undef CHECK_CAP

	const uint width = (stub > 0 ? 0 : cap->run->width);
	const uint height = (stub > 0 ? 0 : cap->run->height);
	const uint hz = (stub > 0 ? 0 : cap->run->hz);
	switch (_drm_find_sink(drm, width, height, hz)) {
		case 0: break;
		case US_ERROR_NO_DEVICE: goto unplugged;
		default: goto error;
	}
	if ((stub == 0) && (width != run->mode.hdisplay || height < run->mode.vdisplay)) {
		// We'll try to show something instead of nothing if height != vdisplay
		stub = US_DRM_STUB_BAD_RESOLUTION;
		_LOG_ERROR("There is no appropriate modes for the capture, forcing to STUB ...");
	}

	if (_drm_init_buffers(drm, (stub > 0 ? NULL : cap)) < 0) {
		goto error;
	}

	run->saved_crtc = drmModeGetCrtc(run->fd, run->crtc_id);
	_LOG_DEBUG("Setting up CRTC ...");

	// Standard CRTC setup for all platforms
	if (drmModeSetCrtc(run->fd, run->crtc_id, run->bufs[0].id, 0, 0, &run->conn_id, 1, &run->mode) < 0) {
		if (errno == EACCES || errno == EPERM) {
			_LOG_INFO("CRTC is busy (probably used by desktop environment), continuing without display control");
		} else {
			_LOG_PERROR("Can't set CRTC");
			goto error;
		}
	}

	_LOG_INFO("Opened for %s ...", (stub > 0 ? "STUB" : "DMA"));
	run->exposing_dma_fd = -1;
	run->blank_at_ts = 0;
	run->opened = stub;
	run->once = 0;
	return run->opened;

error:
	us_drm_close(drm);
	return run->opened; // -1 after us_drm_close()

unplugged:
	US_ONCE_FOR(run->once, __LINE__, {
		_LOG_ERROR("Display is not plugged");
	});
	us_drm_close(drm);
	run->opened = US_ERROR_NO_DEVICE;
	return run->opened;
}

void us_drm_close(us_drm_s *drm) {
	us_drm_runtime_s *const run = drm->run;

	if (run->exposing_dma_fd >= 0) {
		// Нужно подождать, пока dma_fd не освободится, прежде чем прерывать процесс.
		// Просто на всякий случай.
		assert(run->fd >= 0);
		us_drm_wait_for_vsync(drm);
		run->exposing_dma_fd = -1;
	}

	if (run->saved_crtc != NULL) {
		_LOG_DEBUG("Restoring CRTC ...");
		if (drmModeSetCrtc(run->fd,
			run->saved_crtc->crtc_id, run->saved_crtc->buffer_id,
			run->saved_crtc->x, run->saved_crtc->y,
			&run->conn_id, 1, &run->saved_crtc->mode
		) < 0 && errno != ENOENT) {
			_LOG_PERROR("Can't restore CRTC");
		}
		drmModeFreeCrtc(run->saved_crtc);
		run->saved_crtc = NULL;
	}

	if (run->bufs != NULL) {
		_LOG_DEBUG("Releasing buffers ...");
		for (uint n_buf = 0; n_buf < run->n_bufs; ++n_buf) {
			us_drm_buffer_s *const buf = &run->bufs[n_buf];
			if (buf->fb_added && drmModeRmFB(run->fd, buf->id) < 0) {
				_LOG_PERROR("Can't remove buffer=%u", n_buf);
			}
			if (buf->dumb_created) {
				struct drm_mode_destroy_dumb destroy = {.handle = buf->handle};
				if (drmIoctl(run->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy) < 0) {
					_LOG_PERROR("Can't destroy dumb buffer=%u", n_buf);
				}
			}
			if (buf->data != NULL && munmap(buf->data, buf->allocated)) {
				_LOG_PERROR("Can't unmap buffer=%u", n_buf);
			}
		}
		US_DELETE(run->bufs, free);
		run->n_bufs = 0;
	}

	const bool say = (run->fd >= 0);
	US_CLOSE_FD(run->status_fd);

	// Release DRM master control before closing
	if (run->fd >= 0) {
		_LOG_DEBUG("Releasing DRM master control...");
		if (drmDropMaster(run->fd) < 0 && errno != EINVAL) {
			_LOG_DEBUG("Failed to drop DRM master: %s (this might be normal)", strerror(errno));
		}
	}
	US_CLOSE_FD(run->fd);

	run->crtc_id = 0;
	run->dpms_state = -1;
	run->opened = -1;
	run->has_vsync = true;
	run->stub_n_buf = 0;

	if (say) {
		_LOG_INFO("Closed");
	}
}

int us_drm_ensure_no_signal(us_drm_s *drm) {
	us_drm_runtime_s *const run = drm->run;

	assert(run->fd >= 0);
	assert(run->opened > 0);

	const ldf now_ts = us_get_now_monotonic();
	if (run->blank_at_ts == 0) {
		run->blank_at_ts = now_ts + drm->blank_after;
	}
	const ldf saved_ts = run->blank_at_ts; // us_drm*() rewrites it to 0

	int retval;
	if (now_ts <= run->blank_at_ts) {
		retval = us_drm_wait_for_vsync(drm);
		if (retval == 0) {
			retval = us_drm_expose_stub(drm, US_DRM_STUB_NO_SIGNAL, NULL);
		}
	} else {
		US_ONCE_FOR(run->once, __LINE__, {
			_LOG_INFO("Turning off the display by timeout ...");
		});
		retval = us_drm_dpms_power_off(drm);
	}
	run->blank_at_ts = saved_ts;
	return retval;
}

int us_drm_dpms_power_off(us_drm_s *drm) {
	assert(drm->run->fd >= 0);
	switch (_drm_check_status(drm)) {
		case 0: break;
		case US_ERROR_NO_DEVICE: return 0; // Unplugged, nice
		// Во время переключения DPMS монитор моргает один раз состоянием disconnected,
		// а потом почему-то снова оказывается connected. Так что просто считаем,
		// что отсоединенный монитор на этом этапе - это нормально.
		default: return -1;
	}
	_drm_ensure_dpms_power(drm, false);
	return 0;
}

int us_drm_wait_for_vsync(us_drm_s *drm) {
	us_drm_runtime_s *const run = drm->run;

	assert(run->fd >= 0);
	run->blank_at_ts = 0;

	switch (_drm_check_status(drm)) {
		case 0: break;
		case US_ERROR_NO_DEVICE: return US_ERROR_NO_DEVICE;
		default: return -1;
	}
	_drm_ensure_dpms_power(drm, true);

	if (run->has_vsync) {
		return 0;
	}

	// Skip VSync wait for Amlogic platforms - they don't support it reliably
	if (run->platform == US_DRM_PLATFORM_AMLOGIC) {
		_LOG_DEBUG("Skipping VSync wait on Amlogic platform");
		run->has_vsync = true; // Mark as having VSync to avoid future calls
		return 0;
	}

	struct timeval timeout = {.tv_sec = drm->timeout};
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(run->fd, &fds);

	_LOG_DEBUG("Calling select() for VSync ...");
	const int result = select(run->fd + 1, &fds, NULL, NULL, &timeout);
	if (result < 0) {
		_LOG_PERROR("Can't select(%d) device for VSync", run->fd);
		return -1;
	} else if (result == 0) {
		_LOG_ERROR("Device timeout while waiting VSync");
		return -1;
	}

	drmEventContext ctx = {
		.version = DRM_EVENT_CONTEXT_VERSION,
		.page_flip_handler = _drm_vsync_callback,
	};
	_LOG_DEBUG("Handling DRM event (maybe VSync) ...");
	if (drmHandleEvent(run->fd, &ctx) < 0) {
		_LOG_PERROR("Can't handle DRM event");
		return -1;
	}
	return 0;
}

static void _drm_vsync_callback(int fd, uint n_frame, uint sec, uint usec, void *v_buf) {
	(void)fd;
	(void)n_frame;
	(void)sec;
	(void)usec;
	us_drm_buffer_s *const buf = v_buf;
	*buf->ctx.has_vsync = true;
	*buf->ctx.exposing_dma_fd = -1;
	_LOG_DEBUG("Got VSync signal");
}

int us_drm_expose_stub(us_drm_s *drm, us_drm_stub_e stub, const us_capture_s *cap) {
	us_drm_runtime_s *const run = drm->run;

	assert(run->fd >= 0);
	assert(run->opened > 0);
	run->blank_at_ts = 0;

	switch (_drm_check_status(drm)) {
		case 0: break;
		case US_ERROR_NO_DEVICE: return US_ERROR_NO_DEVICE;
		default: return -1;
	}
	_drm_ensure_dpms_power(drm, true);

#	define DRAW_MSG(x_msg) us_frametext_draw(run->ft, (x_msg), run->mode.hdisplay, run->mode.vdisplay)
	switch (stub) {
		case US_DRM_STUB_BAD_RESOLUTION: {
			assert(cap != NULL);
			char msg[1024];
			US_SNPRINTF(msg, 1023,
				"=== PiKVM ==="
				"\n \n< UNSUPPORTED RESOLUTION >"
				"\n \n< %ux%up%.02f >"
				"\n \nby this display",
				cap->run->width, cap->run->height, cap->run->hz);
			DRAW_MSG(msg);
			break;
		};
		case US_DRM_STUB_BAD_FORMAT:
			DRAW_MSG("=== PiKVM ===\n \n< UNSUPPORTED CAPTURE FORMAT >");
			break;
		case US_DRM_STUB_NO_SIGNAL:
			DRAW_MSG("=== PiKVM ===\n \n< NO LIVE VIDEO >");
			break;
		case US_DRM_STUB_BUSY:
			DRAW_MSG("=== PiKVM ===\n \n< ONLINE IS ACTIVE >");
			break;
		default:
			DRAW_MSG("=== PiKVM ===\n \n< ??? >");
			break;
	}
#	undef DRAW_MSG

	us_drm_buffer_s *const buf = &run->bufs[run->stub_n_buf];

	run->has_vsync = false;

	_LOG_DEBUG("Copying STUB frame ...")
	memcpy(buf->data, run->ft->frame->data, US_MIN(run->ft->frame->used, buf->allocated));

	_LOG_DEBUG("Exposing STUB framebuffer n_buf=%u ...", run->stub_n_buf);
	const int retval = drmModePageFlip(
		run->fd, run->crtc_id, buf->id,
		DRM_MODE_PAGE_FLIP_EVENT | DRM_MODE_PAGE_FLIP_ASYNC,
		buf);
	if (retval < 0) {
		if (errno == EACCES || errno == EPERM) {
			_LOG_DEBUG("Page flip permission denied (desktop environment active)");
		} else {
			_LOG_PERROR("Can't expose STUB framebuffer n_buf=%u", run->stub_n_buf);
		}
	}
	_LOG_DEBUG("Exposed STUB framebuffer n_buf=%u", run->stub_n_buf);

	run->stub_n_buf = (run->stub_n_buf + 1) % run->n_bufs;
	return retval;
}

int us_drm_expose_dma(us_drm_s *drm, const us_capture_hwbuf_s *hw) {
	us_drm_runtime_s *const run = drm->run;

	assert(run->fd >= 0);
	assert(run->opened == 0);

	// Safety check for buffer index
	if (hw->buf.index >= run->n_bufs) {
		_LOG_ERROR("Invalid buffer index %u (max: %u)", hw->buf.index, run->n_bufs);
		return -1;
	}

	us_drm_buffer_s *const buf = &run->bufs[hw->buf.index];
	run->blank_at_ts = 0;

	switch (_drm_check_status(drm)) {
		case 0: break;
		case US_ERROR_NO_DEVICE: return US_ERROR_NO_DEVICE;
		default: return -1;
	}
	_drm_ensure_dpms_power(drm, true);

	run->has_vsync = false;

	// If using fallback buffer (DMA import failed), copy data manually
	if (buf->allocated > 0 && buf->data != NULL) {
		_LOG_DEBUG("Copying frame data to fallback framebuffer n_buf=%u ...", hw->buf.index);
		
		// Format conversion with centering for fallback buffer
		if (hw->raw.used > 0 && hw->raw.data != NULL) {
			uint8_t *dst = buf->data;
			const uint dst_width = run->mode.hdisplay;
			const uint dst_height = run->mode.vdisplay;
			
			// Clear the buffer first (black background)
			memset(dst, 0, buf->allocated);
			
			// For MJPEG format, we can't directly process the compressed data
			// Instead, create a simple test pattern or skip processing
			if (hw->raw.format == V4L2_PIX_FMT_MJPEG) {
				_LOG_DEBUG("MJPEG format detected - creating test pattern instead of decoding");
				
				// Create a simple gradient test pattern
				for (uint y = 0; y < dst_height && y * dst_width * 3 < buf->allocated; y++) {
					for (uint x = 0; x < dst_width && (y * dst_width + x) * 3 + 2 < buf->allocated; x++) {
						uint pos = (y * dst_width + x) * 3;
						// Simple gradient pattern: R=x, G=y, B=128
						dst[pos] = (x * 255) / dst_width;      // Red gradient
						dst[pos + 1] = (y * 255) / dst_height; // Green gradient  
						dst[pos + 2] = 128;                    // Blue constant
					}
				}
			} else if (hw->raw.format == V4L2_PIX_FMT_YUYV) {
				// Process YUYV format
				const uint8_t *src = hw->raw.data;
				const uint src_width = hw->raw.width;
				const uint src_height = hw->raw.height;
				
				// Calculate centering offsets
				const uint offset_x = (dst_width > src_width) ? (dst_width - src_width) / 2 : 0;
				const uint offset_y = (dst_height > src_height) ? (dst_height - src_height) / 2 : 0;
				
				_LOG_DEBUG("Centering %ux%u YUYV frame in %ux%u display (offset: %u,%u)", 
					src_width, src_height, dst_width, dst_height, offset_x, offset_y);
				
				// Basic YUYV to RGB conversion with centering
				for (uint y = 0; y < src_height && (y + offset_y) < dst_height; y++) {
					for (uint x = 0; x < src_width && (x + offset_x) < dst_width; x += 2) {
						if ((y * src_width + x) * 2 + 3 < hw->raw.used) {
							int Y1 = src[(y * src_width + x) * 2];
							int U  = src[(y * src_width + x) * 2 + 1];
							int Y2 = src[(y * src_width + x) * 2 + 2];
							int V  = src[(y * src_width + x) * 2 + 3];
							
							// Simple YUV to RGB conversion
							int R1 = Y1 + ((V - 128) * 1436) / 1024;
							int G1 = Y1 - ((U - 128) * 352 + (V - 128) * 731) / 1024;
							int B1 = Y1 + ((U - 128) * 1814) / 1024;
							
							int R2 = Y2 + ((V - 128) * 1436) / 1024;
							int G2 = Y2 - ((U - 128) * 352 + (V - 128) * 731) / 1024;
							int B2 = Y2 + ((U - 128) * 1814) / 1024;
							
							// Clamp values
							R1 = (R1 < 0) ? 0 : (R1 > 255) ? 255 : R1;
							G1 = (G1 < 0) ? 0 : (G1 > 255) ? 255 : G1;
							B1 = (B1 < 0) ? 0 : (B1 > 255) ? 255 : B1;
							R2 = (R2 < 0) ? 0 : (R2 > 255) ? 255 : R2;
							G2 = (G2 < 0) ? 0 : (G2 > 255) ? 255 : G2;
							B2 = (B2 < 0) ? 0 : (B2 > 255) ? 255 : B2;
							
							// Write RGB pixels to centered position
							uint dst_y = y + offset_y;
							uint dst_x = x + offset_x;
							uint dst_pos = dst_y * dst_width + dst_x;
							
							if (dst_pos * 3 + 5 < buf->allocated) {
								dst[dst_pos * 3] = R1;
								dst[dst_pos * 3 + 1] = G1;
								dst[dst_pos * 3 + 2] = B1;
								if (x + 1 < src_width && dst_x + 1 < dst_width) {
									dst[(dst_pos + 1) * 3] = R2;
									dst[(dst_pos + 1) * 3 + 1] = G2;
									dst[(dst_pos + 1) * 3 + 2] = B2;
								}
							}
						}
					}
				}
			} else {
				// For other formats, create a simple test pattern
				_LOG_DEBUG("Unknown format - creating test pattern");
				for (uint y = 0; y < dst_height && y * dst_width * 3 < buf->allocated; y++) {
					for (uint x = 0; x < dst_width && (y * dst_width + x) * 3 + 2 < buf->allocated; x++) {
						uint pos = (y * dst_width + x) * 3;
						dst[pos] = 255;     // Red
						dst[pos + 1] = 0;   // Green  
						dst[pos + 2] = 0;   // Blue
					}
				}
			}
		}
	}

	_LOG_DEBUG("Exposing DMA framebuffer n_buf=%u ...", hw->buf.index);
	const int retval = drmModePageFlip(
		run->fd, run->crtc_id, buf->id,
		DRM_MODE_PAGE_FLIP_EVENT | DRM_MODE_PAGE_FLIP_ASYNC,
		buf);
	if (retval < 0) {
		if (errno == EACCES || errno == EPERM) {
			_LOG_DEBUG("Page flip permission denied (desktop environment active)");
		} else {
			_LOG_PERROR("Can't expose DMA framebuffer n_buf=%u", hw->buf.index);
		}
	}
	_LOG_DEBUG("Exposed DMA framebuffer n_buf=%u", run->stub_n_buf);
	run->exposing_dma_fd = hw->dma_fd;
	return retval;
}

static int _drm_check_status(us_drm_s *drm) {
	us_drm_runtime_s *run = drm->run;

	if (drm->port == NULL) {
		_LOG_DEBUG("Skipping status file check - port not yet detected");
		return 0;
	}

	if (run->status_fd < 0) {
		_LOG_DEBUG("Trying to find status file ...");
		struct stat st;
		if (stat(drm->path, &st) < 0) {
			_LOG_PERROR("Can't stat() DRM device");
			goto error;
		}
		const uint mi = minor(st.st_rdev);
		_LOG_DEBUG("DRM device minor(st_rdev)=%u", mi);

		char path[128];
		US_SNPRINTF(path, 127, "/sys/class/drm/card%u-%s/status", mi, drm->port);
		_LOG_DEBUG("Opening status file %s ...", path);
		if ((run->status_fd = open(path, O_RDONLY | O_CLOEXEC)) < 0) {
			_LOG_PERROR("Can't open status file: %s", path);
			goto error;
		}
		_LOG_DEBUG("Status file fd=%d opened", run->status_fd);
	}

	char status_ch;
	if (read(run->status_fd, &status_ch, 1) != 1) {
		_LOG_PERROR("Can't read status file");
		goto error;
	}
	if (lseek(run->status_fd, 0, SEEK_SET) != 0) {
		_LOG_PERROR("Can't rewind status file");
		goto error;
	}
	_LOG_DEBUG("Current display status: %c", status_ch);
	return (status_ch == 'd' ? US_ERROR_NO_DEVICE : 0);

error:
	US_CLOSE_FD(run->status_fd);
	return -1;
}

static void _drm_ensure_dpms_power(us_drm_s *drm, bool on) {
	us_drm_runtime_s *const run = drm->run;
	if (run->dpms_id > 0 && run->dpms_state != (int)on) {
		_LOG_INFO("Changing DPMS power mode: %d -> %u ...", run->dpms_state, on);
		if (drmModeConnectorSetProperty(
			run->fd, run->conn_id, run->dpms_id,
			(on ? DRM_MODE_DPMS_ON : DRM_MODE_DPMS_OFF)
		) < 0) {
			_LOG_PERROR("Can't set DPMS power=%u (ignored)", on);
		}
	}
	run->dpms_state = (int)on;
}

static int _drm_init_buffers(us_drm_s *drm, const us_capture_s *cap) {
	us_drm_runtime_s *const run = drm->run;

	const uint n_bufs = (cap == NULL ? 4 : cap->run->n_bufs);
	const char *name = (cap == NULL ? "STUB" : "DMA");

	_LOG_DEBUG("Initializing %u %s buffers ...", n_bufs, name);

	uint format = DRM_FORMAT_RGB888;  // Default format
	uint bpp = 24;                    // Default bpp

	// For all platforms, use simplified dumb buffers with better Amlogic compatibility
	if (run->platform == US_DRM_PLATFORM_AMLOGIC) {
		// Amlogic-optimized format but still use dumb buffers (simpler and more reliable)
		format = DRM_FORMAT_XRGB8888;
		bpp = 32;  // Use 32-bit for Amlogic compatibility
		_LOG_INFO("Using Amlogic-optimized dumb buffers: XRGB8888 32-bit");
	} else {
		// Standard format for other platforms
		format = DRM_FORMAT_RGB888;
		bpp = 24;
	}

	US_CALLOC(run->bufs, n_bufs);
	for (run->n_bufs = 0; run->n_bufs < n_bufs; ++run->n_bufs) {
		const uint n_buf = run->n_bufs;
		us_drm_buffer_s *const buf = &run->bufs[n_buf];

		buf->ctx.has_vsync = &run->has_vsync;
		buf->ctx.exposing_dma_fd = &run->exposing_dma_fd;

		u32 handles[4] = {0};
		u32 strides[4] = {0};
		u32 offsets[4] = {0};

		if (cap == NULL) {
			struct drm_mode_create_dumb create = {
				.width = run->mode.hdisplay,
				.height = run->mode.vdisplay,
				.bpp = bpp,
			};
			if (drmIoctl(run->fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) {
				_LOG_PERROR("Can't create %s buffer=%u", name, n_buf);
				return -1;
			}
			buf->handle = create.handle;
			buf->dumb_created = true;

			struct drm_mode_map_dumb map = {.handle = create.handle};
			if (drmIoctl(run->fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) {
				_LOG_PERROR("Can't prepare dumb buffer=%u to mapping", n_buf);
				return -1;
			}
			if ((buf->data = mmap(
				NULL, create.size,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				run->fd, map.offset
			)) == MAP_FAILED) {
				_LOG_PERROR("Can't map buffer=%u", n_buf);
				return -1;
			}
			memset(buf->data, 0, create.size);
			buf->allocated = create.size;

			handles[0] = create.handle;
			strides[0] = create.pitch;

		} else {
			// Skip DMA-BUF import for Amlogic platforms - use direct buffer copy instead
			bool use_dma_import = false;
			if (run->platform != US_DRM_PLATFORM_AMLOGIC) {
				_LOG_DEBUG("Attempting DMA buffer import for buffer %u", n_buf);
				if (drmPrimeFDToHandle(run->fd, cap->run->bufs[n_buf].dma_fd, &buf->handle) >= 0) {
					use_dma_import = true;
					_LOG_DEBUG("DMA buffer import successful for buffer %u", n_buf);
				} else {
					_LOG_DEBUG("DMA-BUF import failed for buffer %u: %s", n_buf, strerror(errno));
				}
			} else {
				_LOG_DEBUG("Amlogic platform detected, skipping DMA-BUF import for buffer %u", n_buf);
			}

			if (!use_dma_import) {
				_LOG_DEBUG("Using manual buffer creation for buffer %u", n_buf);

				// Manual buffer creation for all failed DMA imports or Amlogic
				struct drm_mode_create_dumb create = {0};
				create.width = run->mode.hdisplay;   // Use display resolution, not capture resolution
				create.height = run->mode.vdisplay;  // Use display resolution, not capture resolution
				// For Amlogic platform with XRGB8888, we need 32-bit buffer
				create.bpp = (run->platform == US_DRM_PLATFORM_AMLOGIC) ? 32 : 24;

				_LOG_DEBUG("Creating fallback dumb buffer: %ux%u, bpp=%u",
				          create.width, create.height, create.bpp);

				if (drmIoctl(run->fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) {
					_LOG_PERROR("Can't create fallback dumb buffer=%u", n_buf);
					return -1;
				}
				buf->handle = create.handle;
				buf->dumb_created = true;

				struct drm_mode_map_dumb map = {0};
				map.handle = create.handle;
				if (drmIoctl(run->fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) {
					_LOG_PERROR("Can't get fallback buffer=%u map info", n_buf);
					// Clean up on error
					struct drm_mode_destroy_dumb destroy = {.handle = create.handle};
					drmIoctl(run->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
					return -1;
				}
				
				if ((buf->data = mmap(NULL, create.size, PROT_READ | PROT_WRITE, MAP_SHARED,
					run->fd, map.offset
				)) == MAP_FAILED) {
					_LOG_PERROR("Can't map fallback buffer=%u", n_buf);
					// Clean up on error
					struct drm_mode_destroy_dumb destroy = {.handle = create.handle};
					drmIoctl(run->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
					return -1;
				}
				memset(buf->data, 0, create.size);
				buf->allocated = create.size;
				
				handles[0] = create.handle;
				strides[0] = create.pitch;
				// Format is already set based on platform at the beginning of this function
				_LOG_DEBUG("Created fallback buffer: %ux%u, bpp=%u, pitch=%u, size=%llu, handle=%u",
					create.width, create.height, create.bpp, create.pitch, (unsigned long long)create.size, create.handle);
			} else {
				handles[0] = buf->handle;
				strides[0] = cap->run->stride;
				
				switch (cap->run->format) {
					case V4L2_PIX_FMT_RGB24: format = (DRM_FORMAT_BIG_ENDIAN ? DRM_FORMAT_BGR888 : DRM_FORMAT_RGB888); break;
					case V4L2_PIX_FMT_BGR24: format = (DRM_FORMAT_BIG_ENDIAN ? DRM_FORMAT_RGB888 : DRM_FORMAT_BGR888); break;
					case V4L2_PIX_FMT_YUYV: format = DRM_FORMAT_YUYV; break;
					case V4L2_PIX_FMT_MJPEG:
						// MJPEG needs to be decoded first, use XRGB8888 as fallback
						format = DRM_FORMAT_XRGB8888;
						_LOG_INFO("MJPEG format detected, will decode to XRGB8888 for display");
						break;
				}
			}
		}

		int fb_ret = -1;
		if (cap == NULL) {
			// For STUB buffers, use different methods based on platform
			if (run->platform == US_DRM_PLATFORM_AMLOGIC) {
				// Amlogic: Use legacy AddFB for better compatibility
				// For XRGB8888: depth=24 (RGB without padding), bpp=32 (4 bytes per pixel)
				uint fb_depth = 24;
				uint fb_bpp = 32;
				_LOG_DEBUG("Creating Amlogic STUB framebuffer: %ux%u, depth=%u, bpp=%u, handle=%u, stride=%u",
					run->mode.hdisplay, run->mode.vdisplay, fb_depth, fb_bpp, handles[0], strides[0]);
				fb_ret = drmModeAddFB(run->fd, run->mode.hdisplay, run->mode.vdisplay,
				                     fb_depth, fb_bpp, strides[0], handles[0], &buf->id);
			} else {
				// Other platforms: Use AddFB2 method
				_LOG_DEBUG("Creating STUB framebuffer: %ux%u, format=0x%x, handle=%u, stride=%u",
					run->mode.hdisplay, run->mode.vdisplay, format, handles[0], strides[0]);
				fb_ret = drmModeAddFB2(run->fd, run->mode.hdisplay, run->mode.vdisplay,
				                       format, handles, strides, offsets, &buf->id, 0);
			}
			
			if (fb_ret != 0) {
				// Original method failed, try fallback formats
				_LOG_DEBUG("Original RGB888 format failed, trying fallback formats...");
				struct {
					uint bpp;
					const char *name;
				} fallback_formats[] = {
					{32, "XRGB8888"},
					{16, "RGB565"},
				};
				
				bool fallback_success = false;
				for (int i = 0; i < 2 && !fallback_success; i++) {
					// Recreate buffer with different bpp
					if (buf->dumb_created) {
						struct drm_mode_destroy_dumb destroy_old = {.handle = buf->handle};
						drmIoctl(run->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy_old);
						if (buf->data != NULL) {
							munmap(buf->data, buf->allocated);
							buf->data = NULL;
						}
						buf->dumb_created = false;
					}
					
					struct drm_mode_create_dumb create_fb = {
						.width = run->mode.hdisplay,
						.height = run->mode.vdisplay,
						.bpp = fallback_formats[i].bpp,
					};
					
					if (drmIoctl(run->fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_fb) == 0) {
						buf->handle = create_fb.handle;
						buf->dumb_created = true;
						
						// Map the buffer
						struct drm_mode_map_dumb map_fb = {.handle = create_fb.handle};
						if (drmIoctl(run->fd, DRM_IOCTL_MODE_MAP_DUMB, &map_fb) == 0) {
							buf->data = mmap(NULL, create_fb.size, PROT_READ | PROT_WRITE, 
							               MAP_SHARED, run->fd, map_fb.offset);
							if (buf->data != MAP_FAILED) {
								memset(buf->data, 0, create_fb.size);
								buf->allocated = create_fb.size;
								
								// Try to create framebuffer with legacy API
								handles[0] = create_fb.handle;
								strides[0] = create_fb.pitch;
								fb_ret = drmModeAddFB(run->fd, run->mode.hdisplay, run->mode.vdisplay, 
								                     fallback_formats[i].bpp, fallback_formats[i].bpp, 
								                     create_fb.pitch, create_fb.handle, &buf->id);
								
								if (fb_ret == 0) {
									_LOG_INFO("Successfully using fallback format: %s (%u bpp)", 
									         fallback_formats[i].name, fallback_formats[i].bpp);
									bpp = fallback_formats[i].bpp;  // 记录成功的bpp供后续使用
									run->detected_bpp = bpp;         // 保存到运行时状态
									fallback_success = true;
								}
							}
						}
					}
				}
				
				if (!fallback_success) {
					fb_ret = -1; // 确保失败状态
				}
			}
		} else {
			// For DMA buffers, use platform-specific API
			if (run->platform == US_DRM_PLATFORM_AMLOGIC) {
				// Amlogic: Use legacy AddFB for DMA buffers too
				// For XRGB8888: depth=24, bpp=32
				uint fb_depth = 24;
				uint fb_bpp = 32;
				_LOG_DEBUG("Creating Amlogic DMA framebuffer: %ux%u, depth=%u, bpp=%u, handle=%u, stride=%u",
					cap->run->width, cap->run->height, fb_depth, fb_bpp, handles[0], strides[0]);
				fb_ret = drmModeAddFB(run->fd, cap->run->width, cap->run->height,
				                     fb_depth, fb_bpp, strides[0], handles[0], &buf->id);
			} else {
				// Other platforms: Use AddFB2 with format-specific handling
				_LOG_DEBUG("Creating DMA framebuffer: %ux%u, format=0x%x, handle=%u, stride=%u",
					cap->run->width, cap->run->height, format, handles[0], strides[0]);
				fb_ret = drmModeAddFB2(run->fd, cap->run->width, cap->run->height,
				                       format, handles, strides, offsets, &buf->id, 0);
			}
		}
		
		if (fb_ret) {
			_LOG_PERROR("Can't setup buffer=%u", n_buf);
			return -1;
		}
		buf->fb_added = true;
	}
	return 0;
}

static int _drm_find_sink(us_drm_s *drm, uint width, uint height, float hz) {
	us_drm_runtime_s *const run = drm->run;

	run->crtc_id = 0;

	_LOG_DEBUG("Trying to find the appropriate sink ...");

	drmModeRes *res = drmModeGetResources(run->fd);
	if (res == NULL) {
		_LOG_PERROR("Can't get resources info");
		goto done;
	}
	if (res->count_connectors <= 0) {
		_LOG_ERROR("Can't find any connectors");
		goto done;
	}

	for (int ci = 0; ci < res->count_connectors; ++ci) {
		drmModeConnector *conn = drmModeGetConnector(run->fd, res->connectors[ci]);
		if (conn == NULL) {
			_LOG_PERROR("Can't get connector index=%d", ci);
			goto done;
		}

		char port[32];
		US_SNPRINTF(port, 31, "%s-%u",
			_connector_type_to_string(conn->connector_type),
			conn->connector_type_id);

		if (drm->port != NULL && strcmp(port, drm->port) != 0) {
			drmModeFreeConnector(conn);
			continue;
		}

		if (drm->port == NULL) {
			if (conn->connection != DRM_MODE_CONNECTED) {
				drmModeFreeConnector(conn);
				continue;
			}
			drm->port = strdup(port);
			_LOG_INFO("Auto-detected connector %s: conn_type=%d, conn_type_id=%d",
				port, conn->connector_type, conn->connector_type_id);
		} else {
			_LOG_INFO("Using connector %s: conn_type=%d, conn_type_id=%d",
				drm->port, conn->connector_type, conn->connector_type_id);
		}

		if (conn->connection != DRM_MODE_CONNECTED) {
			_LOG_ERROR("Connector for port %s has !DRM_MODE_CONNECTED", port);
			drmModeFreeConnector(conn);
			goto done;
		}

		const drmModeModeInfo *best;
		if ((best = _find_best_mode(conn, width, height, hz)) == NULL) {
			_LOG_ERROR("Can't find any appropriate display modes");
			drmModeFreeConnector(conn);
			goto unplugged;
		}
		_LOG_INFO("Using best mode: %ux%up%.02f",
			best->hdisplay, best->vdisplay, _get_refresh_rate(best));

		if ((run->dpms_id = _find_dpms(run->fd, conn)) > 0) {
			_LOG_INFO("Using DPMS: id=%u", run->dpms_id);
		} else {
			_LOG_INFO("Using DPMS: None");
		}

		u32 taken_crtcs = 0; // Unused here
		if ((run->crtc_id = _find_crtc(run->fd, res, conn, &taken_crtcs)) == 0) {
			_LOG_ERROR("Can't find CRTC");
			drmModeFreeConnector(conn);
			goto done;
		}
		_LOG_INFO("Using CRTC: id=%u", run->crtc_id);

		run->conn_id = conn->connector_id;
		memcpy(&run->mode, best, sizeof(drmModeModeInfo));

		// Pre-calculate display stride for performance (XRGB8888 = 4 bytes per pixel)
		run->display_stride = run->mode.hdisplay * 4;
		_LOG_DEBUG("Pre-calculated display stride: %u (hdisplay=%u)", run->display_stride, run->mode.hdisplay);

		drmModeFreeConnector(conn);
		break;
	}

done:
	drmModeFreeResources(res);
	return (run->crtc_id > 0 ? 0 : -1);

unplugged:
	drmModeFreeResources(res);
	return US_ERROR_NO_DEVICE;
}

static drmModeModeInfo *_find_best_mode(drmModeConnector *conn, uint width, uint height, float hz) {
	drmModeModeInfo *best = NULL;
	drmModeModeInfo *closest = NULL;
	drmModeModeInfo *pref = NULL;

	for (int mi = 0; mi < conn->count_modes; ++mi) {
		drmModeModeInfo *const mode = &conn->modes[mi];
		if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
			continue; // Discard interlaced
		}
		const float mode_hz = _get_refresh_rate(mode);
		if (width == 640 && height == 416 && mode->hdisplay == 640 && mode->vdisplay == 480) {
			// A special case for some ancient DOS device with VGA converter.
			// @CapnKirk in Discord
			if (hz > 0 && mode_hz < hz) {
				best = mode;
				best->vdisplay = 416;
				break;
			}
		}
		if (mode->hdisplay == width && mode->vdisplay == height) {
			best = mode; // Any mode with exact resolution
			if (hz > 0 && mode_hz == hz) {
				break; // Exact mode with same freq
			}
		}
		if (mode->hdisplay == width && mode->vdisplay < height) {
			if (closest == NULL || _get_refresh_rate(closest) != hz) {
				closest = mode; // Something like 1920x1080p60 for 1920x1200p60 source
			}
		}
		if (pref == NULL && (mode->type & DRM_MODE_TYPE_PREFERRED)) {
			pref = mode; // Preferred mode if nothing is found
		}
	}

	if (best == NULL) {
		best = closest;
	}
	if (best == NULL) {
		best = pref;
	}
	if (best == NULL) {
		best = (conn->count_modes > 0 ? &conn->modes[0] : NULL);
	}
	assert(best == NULL || best->hdisplay > 0);
	assert(best == NULL || best->vdisplay > 0);
	return best;
}

static u32 _find_dpms(int fd, drmModeConnector *conn) {
	for (int pi = 0; pi < conn->count_props; pi++) {
		drmModePropertyPtr prop = drmModeGetProperty(fd, conn->props[pi]);
		if (prop != NULL) {
			if (!strcmp(prop->name, "DPMS")) {
				const u32 id = prop->prop_id;
				drmModeFreeProperty(prop);
				return id;
			}
			drmModeFreeProperty(prop);
		}
	}
	return 0;
}

static u32 _find_crtc(int fd, drmModeRes *res, drmModeConnector *conn, u32 *taken_crtcs) {
	for (int ei = 0; ei < conn->count_encoders; ++ei) {
		drmModeEncoder *enc = drmModeGetEncoder(fd, conn->encoders[ei]);
		if (enc == NULL) {
			continue;
		}
		for (int ci = 0; ci < res->count_crtcs; ++ci) {
			u32 bit = (1 << ci);
			if (!(enc->possible_crtcs & bit)) {
				continue; // Not compatible
			}
			if (*taken_crtcs & bit) {
				continue; // Already taken
			}
			drmModeFreeEncoder(enc);
			*taken_crtcs |= bit;
			return res->crtcs[ci];
		}
		drmModeFreeEncoder(enc);
	}
	return 0;
}

static const char *_connector_type_to_string(u32 type) {
	switch (type) {
#		define CASE_NAME(x_suffix, x_name) \
			case DRM_MODE_CONNECTOR_##x_suffix: return x_name;
		CASE_NAME(VGA,			"VGA");
		CASE_NAME(DVII,			"DVI-I");
		CASE_NAME(DVID,			"DVI-D");
		CASE_NAME(DVIA,			"DVI-A");
		CASE_NAME(Composite,	"Composite");
		CASE_NAME(SVIDEO,		"SVIDEO");
		CASE_NAME(LVDS,			"LVDS");
		CASE_NAME(Component,	"Component");
		CASE_NAME(9PinDIN,		"DIN");
		CASE_NAME(DisplayPort,	"DP");
		CASE_NAME(HDMIA,		"HDMI-A");
		CASE_NAME(HDMIB,		"HDMI-B");
		CASE_NAME(TV,			"TV");
		CASE_NAME(eDP,			"eDP");
		CASE_NAME(VIRTUAL,		"Virtual");
		CASE_NAME(DSI,			"DSI");
		CASE_NAME(DPI,			"DPI");
		CASE_NAME(WRITEBACK,	"Writeback");
		CASE_NAME(SPI,			"SPI");
		CASE_NAME(USB,			"USB");
		case DRM_MODE_CONNECTOR_Unknown: break;
#		undef CASE_NAME
	}
	return "Unknown";
}

static float _get_refresh_rate(const drmModeModeInfo *mode) {
	int mhz = (mode->clock * 1000000LL / mode->htotal + mode->vtotal / 2) / mode->vtotal;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		mhz *= 2;
	}
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN) {
		mhz /= 2;
	}
	if (mode->vscan > 1) {
		mhz /= mode->vscan;
	}
	return (float)mhz / 1000;
}

static void _drm_calculate_center(us_drm_center_s *center, uint src_w, uint src_h, uint dst_w, uint dst_h) {
	center->src_width = src_w;
	center->src_height = src_h;
	center->dst_width = dst_w;
	center->dst_height = dst_h;
	
	if (src_w <= dst_w && src_h <= dst_h) {
		center->offset_x = (dst_w - src_w) / 2;
		center->offset_y = (dst_h - src_h) / 2;
		center->needs_center = true;
		_LOG_DEBUG("Centering: %ux%u -> %ux%u, offset=(%u,%u)", 
			src_w, src_h, dst_w, dst_h, center->offset_x, center->offset_y);
	} else {
		center->offset_x = 0;
		center->offset_y = 0;
		center->needs_center = false;
		_LOG_DEBUG("No centering needed: source %ux%u >= display %ux%u", src_w, src_h, dst_w, dst_h);
	}
}

int us_drm_expose_centered(us_drm_s *drm, const us_capture_hwbuf_s *hw) {
	us_drm_runtime_s *const run = drm->run;

	// Common pre-checks for all platforms
	assert(run->fd >= 0);
	assert(run->opened == 0);
	run->blank_at_ts = 0;

	switch (_drm_check_status(drm)) {
		case 0: break;
		case US_ERROR_NO_DEVICE: return US_ERROR_NO_DEVICE;
		default: return -1;
	}
	_drm_ensure_dpms_power(drm, true);

	// Dispatch to platform-specific implementation
	switch (run->platform) {
		case US_DRM_PLATFORM_RPI:
			return _drm_expose_rpi_v4p_impl(drm, hw);
		case US_DRM_PLATFORM_AMLOGIC:
			return _drm_expose_amlogic_display_impl(drm, hw);
		default:
			_LOG_ERROR("Unsupported DRM platform for centered display");
			return -1;
	}
}

static us_drm_platform_e _detect_drm_platform(int fd) {
	drmVersionPtr version = drmGetVersion(fd);
	if (!version) {
		_LOG_DEBUG("Can't get DRM version, using generic platform");
		return US_DRM_PLATFORM_GENERIC;
	}

	us_drm_platform_e platform = US_DRM_PLATFORM_GENERIC;

	_LOG_DEBUG("DRM driver: %s, version: %d.%d.%d",
		version->name, version->version_major,
		version->version_minor, version->version_patchlevel);

	if (version->name) {
		if (strstr(version->name, "vc4") != NULL) {
			// Raspberry Pi VideoCore IV
			platform = US_DRM_PLATFORM_RPI;
		} else if (strstr(version->name, "meson") != NULL) {
			// Amlogic Meson (S805, S912, etc.)
			platform = US_DRM_PLATFORM_AMLOGIC;
		}
		// Add other platform detection as needed
	}

	drmFreeVersion(version);
	return platform;
}

static void _drm_convert_yuyv_simple(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	(void)dst_bpp; (void)dst_w; (void)dst_h;
	u8 *dst_offset = dst_data + (center->offset_y * dst_stride) + (center->offset_x * 4);
	YUY2ToARGB(src_data, src_w * 2, dst_offset, dst_stride, src_w, src_h);
}


static void _drm_convert_rgb24(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	const uint bytes_per_pixel = dst_bpp / 8;
	const uint src_stride = src_w * 3; // RGB24 = 3 bytes per pixel

	for (uint y = 0; y < src_h && y < center->src_height && (y + center->offset_y) < dst_h; y++) {
		const u8 *src_row = src_data + (y * src_stride);
		u8 *dst_row = dst_data + ((y + center->offset_y) * dst_stride) + (center->offset_x * bytes_per_pixel);

		for (uint x = 0; x < src_w && x < center->src_width && (x + center->offset_x) < dst_w; x++) {
			const uint src_pos = x * 3;
			if (src_pos + 2 >= src_stride) break;

			u8 R = src_row[src_pos + 0];
			u8 G = src_row[src_pos + 1];
			u8 B = src_row[src_pos + 2];

			u8 *dst_pixel = dst_row + (x * bytes_per_pixel);
			if (bytes_per_pixel == 4) {
				dst_pixel[0] = B; dst_pixel[1] = G; dst_pixel[2] = R; dst_pixel[3] = 0xFF;
			} else {
				dst_pixel[0] = R; dst_pixel[1] = G; dst_pixel[2] = B;
			}
		}
	}
}

static void _drm_convert_bgr24(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	const uint bytes_per_pixel = dst_bpp / 8;
	const uint src_stride = src_w * 3;

	for (uint y = 0; y < src_h && y < center->src_height && (y + center->offset_y) < dst_h; y++) {
		const u8 *src_row = src_data + (y * src_stride);
		u8 *dst_row = dst_data + ((y + center->offset_y) * dst_stride) + (center->offset_x * bytes_per_pixel);

		for (uint x = 0; x < src_w && x < center->src_width && (x + center->offset_x) < dst_w; x++) {
			const uint src_pos = x * 3;
			if (src_pos + 2 >= src_stride) break;

			u8 B = src_row[src_pos + 0];
			u8 G = src_row[src_pos + 1];
			u8 R = src_row[src_pos + 2];

			u8 *dst_pixel = dst_row + (x * bytes_per_pixel);
			if (bytes_per_pixel == 4) {
				dst_pixel[0] = B; dst_pixel[1] = G; dst_pixel[2] = R; dst_pixel[3] = 0xFF;
			} else {
				dst_pixel[0] = R; dst_pixel[1] = G; dst_pixel[2] = B;
			}
		}
	}
}

static void _drm_convert_mjpeg(const u8 *src_data, size_t src_size, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	_LOG_DEBUG("Decoding MJPEG frame %ux%u (%zu bytes)", src_w, src_h, src_size);

	// Create temporary frames for JPEG decoding
	us_frame_s src_frame = {0};
	us_frame_s decoded_frame = {0};

	// Setup source frame with MJPEG data
	src_frame.width = src_w;
	src_frame.height = src_h;
	src_frame.format = V4L2_PIX_FMT_MJPEG;
	src_frame.used = src_size;
	src_frame.allocated = src_size;
	src_frame.data = (u8*)src_data; // Cast away const for compatibility
	src_frame.grab_ts = us_get_now_monotonic();

	// Try to decode MJPEG to RGB24
	if (us_unjpeg(&src_frame, &decoded_frame, true) == 0 && decoded_frame.data != NULL) {
		_LOG_DEBUG("MJPEG decoded successfully to %ux%u RGB24", decoded_frame.width, decoded_frame.height);

		// Now convert the decoded RGB24 to display format
		_drm_convert_rgb24(decoded_frame.data, decoded_frame.width, decoded_frame.height,
			dst_data, center, dst_stride, dst_bpp, dst_w, dst_h);

		// Clean up decoded frame
		US_DELETE(decoded_frame.data, free);
	} else {
		_LOG_ERROR("MJPEG decoding failed, cannot display frame");
		// Leave buffer black (already cleared) - don't show fake data
	}
}


// ============================================================================
// Platform-specific implementations
// ============================================================================

// Original Raspberry Pi V4P DRM implementation - uses DMA buffer import
static int _drm_expose_rpi_v4p_impl(us_drm_s *drm, const us_capture_hwbuf_s *hw) {
	us_drm_runtime_s *const run = drm->run;

	// Calculate centering parameters
	us_drm_center_s center;
	_drm_calculate_center(&center,
		hw->raw.width, hw->raw.height,
		run->mode.hdisplay, run->mode.vdisplay);

	// Safety checks
	if (hw->buf.index >= run->n_bufs) {
		_LOG_ERROR("Invalid buffer index %u (max: %u)", hw->buf.index, run->n_bufs);
		return -1;
	}

	us_drm_buffer_s *const buf = &run->bufs[hw->buf.index];
	if (!buf->data) {
		_LOG_ERROR("Buffer data is NULL for buffer %u", hw->buf.index);
		return -1;
	}

	run->has_vsync = false;

	_LOG_DEBUG("RPI V4P: Exposing DMA buffer %u", hw->buf.index);

	// Original V4P uses page flip with DMA buffers
	const int retval = drmModePageFlip(
		run->fd, run->crtc_id, buf->id,
		DRM_MODE_PAGE_FLIP_EVENT, buf);

	if (retval < 0) {
		if (errno == EACCES || errno == EPERM) {
			_LOG_DEBUG("Page flip permission denied (desktop environment active)");
		} else {
			_LOG_PERROR("Can't expose V4P framebuffer n_buf=%u", hw->buf.index);
		}
	} else {
		_LOG_DEBUG("V4P framebuffer exposed successfully");
	}

	return retval;
}

// New Amlogic display implementation - uses format conversion and SetCRTC
static int _drm_expose_amlogic_display_impl(us_drm_s *drm, const us_capture_hwbuf_s *hw) {
	us_drm_runtime_s *const run = drm->run;

	// Calculate centering parameters
	us_drm_center_s center;
	_drm_calculate_center(&center,
		hw->raw.width, hw->raw.height,
		run->mode.hdisplay, run->mode.vdisplay);

	if (!center.needs_center) {
		_LOG_ERROR("Source resolution %ux%u is larger than display %ux%u",
			hw->raw.width, hw->raw.height, run->mode.hdisplay, run->mode.vdisplay);
		return us_drm_expose_stub(drm, US_DRM_STUB_BAD_RESOLUTION, NULL);
	}

	// Safety checks
	if (hw->buf.index >= run->n_bufs) {
		_LOG_ERROR("Invalid buffer index %u (max: %u)", hw->buf.index, run->n_bufs);
		return -1;
	}

	us_drm_buffer_s *const buf = &run->bufs[hw->buf.index];
	if (!buf->data || buf->allocated == 0) {
		_LOG_ERROR("Invalid buffer %u for Amlogic display", hw->buf.index);
		return -1;
	}

	run->has_vsync = false;

	// Check for valid frame data
	if (!hw->raw.data || hw->raw.used == 0) {
		_LOG_DEBUG("No valid frame data available, skipping display");
		return -1;
	}

	// Clear buffer only if resolution changed (for borders)
	// For full-screen display, skip memset - libyuv will overwrite entire buffer
	static uint last_src_w = 0, last_src_h = 0;
	const bool resolution_changed = (last_src_w != hw->raw.width || last_src_h != hw->raw.height);
	const bool is_fullscreen = (hw->raw.width == run->mode.hdisplay && hw->raw.height == run->mode.vdisplay);

	if (resolution_changed && !is_fullscreen) {
		memset(buf->data, 0, buf->allocated);
		last_src_w = hw->raw.width;
		last_src_h = hw->raw.height;
	}

	// Convert frame to display format
	const uint actual_bpp = 32; // XRGB8888 for Amlogic
	const uint dst_stride = run->display_stride; // Pre-calculated in init

	// Log frame conversion only once per second to reduce overhead
	static uint frame_counter = 0;
	static uint last_logged_format = 0;
	const bool should_log = ((frame_counter++ % 60) == 0) || (last_logged_format != hw->raw.format);

	if (should_log) {
		const char *format_name = "UNKNOWN";
		switch (hw->raw.format) {
			case V4L2_PIX_FMT_YUYV: format_name = "YUYV"; break;
			case V4L2_PIX_FMT_RGB24: format_name = "RGB24"; break;
			case V4L2_PIX_FMT_BGR24: format_name = "BGR24"; break;
			case V4L2_PIX_FMT_MJPEG: format_name = "MJPEG"; break;
			case V4L2_PIX_FMT_JPEG: format_name = "JPEG"; break;
		}
		_LOG_DEBUG("Amlogic: Converting %s %ux%u → display %ux%u",
			format_name, hw->raw.width, hw->raw.height, run->mode.hdisplay, run->mode.vdisplay);
		last_logged_format = hw->raw.format;
	}

	switch (hw->raw.format) {
		case V4L2_PIX_FMT_YUYV:
			_drm_convert_yuyv_simple(hw->raw.data, hw->raw.width, hw->raw.height,
				buf->data, &center, dst_stride, actual_bpp, run->mode.hdisplay, run->mode.vdisplay);
			break;

		case V4L2_PIX_FMT_RGB24:
			_drm_convert_rgb24(hw->raw.data, hw->raw.width, hw->raw.height,
				buf->data, &center, dst_stride, actual_bpp, run->mode.hdisplay, run->mode.vdisplay);
			break;

		case V4L2_PIX_FMT_BGR24:
			_drm_convert_bgr24(hw->raw.data, hw->raw.width, hw->raw.height,
				buf->data, &center, dst_stride, actual_bpp, run->mode.hdisplay, run->mode.vdisplay);
			break;

		case V4L2_PIX_FMT_MJPEG:
		case V4L2_PIX_FMT_JPEG:
			_drm_convert_mjpeg(hw->raw.data, hw->raw.used, hw->raw.width, hw->raw.height,
				buf->data, &center, dst_stride, actual_bpp, run->mode.hdisplay, run->mode.vdisplay);
			break;

		default:
			_LOG_ERROR("Unsupported format 0x%08x for Amlogic display", hw->raw.format);
			return -1;
	}

	// Display using SetCRTC (more compatible than page flip for Amlogic)
	const int retval = drmModeSetCrtc(run->fd, run->crtc_id, buf->id,
		0, 0, &run->conn_id, 1, &run->mode);

	if (retval < 0) {
		_LOG_PERROR("Can't set CRTC for Amlogic framebuffer n_buf=%u", hw->buf.index);
	} else {
		_LOG_DEBUG("Amlogic framebuffer displayed successfully");
	}

	return retval;
}


// Public API functions for platform-specific implementations
int us_drm_expose_rpi_v4p(us_drm_s *drm, const us_capture_hwbuf_s *hw) {
	return _drm_expose_rpi_v4p_impl(drm, hw);
}

int us_drm_expose_amlogic_display(us_drm_s *drm, const us_capture_hwbuf_s *hw) {
	return _drm_expose_amlogic_display_impl(drm, hw);
}
