// Optimized DRM format conversion functions using libyuv

static void _drm_convert_yuyv_simple(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	u8 *dst_offset = dst_data + (center->offset_y * dst_stride) + (center->offset_x * 4);
	YUY2ToARGB(src_data, src_w * 2, dst_offset, dst_stride, src_w, src_h);
}

static void _drm_convert_rgb24(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	u8 *dst_offset = dst_data + (center->offset_y * dst_stride) + (center->offset_x * 4);
	RGB24ToARGB(src_data, src_w * 3, dst_offset, dst_stride, src_w, src_h);
}

static void _drm_convert_bgr24(const u8 *src_data, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	u8 *dst_offset = dst_data + (center->offset_y * dst_stride) + (center->offset_x * 4);
	// libyuv uses ARGB format (B G R A in little-endian), which matches XRGB8888
	RAWToARGB(src_data, src_w * 3, dst_offset, dst_stride, src_w, src_h);
}

static void _drm_convert_mjpeg(const u8 *src_data, size_t src_size, uint src_w, uint src_h, u8 *dst_data, const us_drm_center_s *center, uint dst_stride, uint dst_bpp, uint dst_w, uint dst_h) {
	u8 *tmp_rgb = malloc(src_w * src_h * 3);
	if (tmp_rgb == NULL) {
		_LOG_ERROR("Failed to allocate temporary buffer for MJPEG decoding");
		return;
	}

	// Decode MJPEG to RGB24 using libyuv
	if (MJPEGToI420(src_data, src_size,
	                tmp_rgb, src_w,
	                tmp_rgb + (src_w * src_h), src_w / 2,
	                tmp_rgb + (src_w * src_h * 5 / 4), src_w / 2,
	                src_w, src_h, src_w, src_h) == 0) {
		// Convert I420 to ARGB
		u8 *dst_offset = dst_data + (center->offset_y * dst_stride) + (center->offset_x * 4);
		I420ToARGB(tmp_rgb, src_w,
		           tmp_rgb + (src_w * src_h), src_w / 2,
		           tmp_rgb + (src_w * src_h * 5 / 4), src_w / 2,
		           dst_offset, dst_stride, src_w, src_h);
	} else {
		_LOG_ERROR("MJPEG decoding failed");
	}

	free(tmp_rgb);
}