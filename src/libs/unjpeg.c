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


#include "unjpeg.h"

#include <stdio.h>
#include <setjmp.h>
#include <assert.h>

#include <jpeglib.h>
#include <linux/videodev2.h>

#include "types.h"
#include "logging.h"
#include "frame.h"


typedef struct {
	struct jpeg_error_mgr	mgr; // Default manager
	jmp_buf					jmp;
	const us_frame_s		*frame;
} _jpeg_error_manager_s;


static void _jpeg_error_handler(j_common_ptr jpeg);


int us_unjpeg(const us_frame_s *src, us_frame_s *dest, bool decode) {
	assert(us_is_jpeg(src->format));

	volatile int retval = 0;

	struct jpeg_decompress_struct jpeg;
	jpeg_create_decompress(&jpeg);

	// https://stackoverflow.com/questions/19857766/error-handling-in-libjpeg
	_jpeg_error_manager_s jpeg_error;
	jpeg.err = jpeg_std_error((struct jpeg_error_mgr*)&jpeg_error);
	jpeg_error.mgr.error_exit = _jpeg_error_handler;
	jpeg_error.frame = src;
	if (setjmp(jpeg_error.jmp) < 0) {
		retval = -1;
		goto done;
	}

    jpeg_mem_src(&jpeg, src->data, src->used);
    jpeg_read_header(&jpeg, TRUE);

    // 优先尝试以 YUV420 原始平面输出，避免 RGB 中转
    bool yuv420_ok = false;
    if (decode) {
        // 仅在采样为 4:2:0 时走 raw_data_out；否则回退到 RGB
        if (jpeg.num_components == 3) {
            jpeg_component_info *ciY  = &jpeg.comp_info[0];
            jpeg_component_info *ciCb = &jpeg.comp_info[1];
            jpeg_component_info *ciCr = &jpeg.comp_info[2];
            const bool is_420 = (ciY->h_samp_factor == 2 && ciY->v_samp_factor == 2 &&
                                 ciCb->h_samp_factor == 1 && ciCb->v_samp_factor == 1 &&
                                 ciCr->h_samp_factor == 1 && ciCr->v_samp_factor == 1);
            if (is_420) {
                jpeg.raw_data_out = TRUE;
                jpeg.out_color_space = JCS_YCbCr;
                jpeg_start_decompress(&jpeg);

                US_FRAME_COPY_META(src, dest);
                dest->format = V4L2_PIX_FMT_YUV420; // I420: Y + U + V
                dest->width = jpeg.output_width;
                dest->height = jpeg.output_height;
                dest->stride = jpeg.output_width;

                const uz y_size  = (uz)dest->width * dest->height;
                const uz uv_w    = dest->width / 2;
                const uz uv_h    = dest->height / 2;
                const uz c_size  = uv_w * uv_h;
                const uz need    = y_size + 2 * c_size;
                us_frame_realloc_data(dest, need);
                dest->used = need;

                u8 *y_plane  = dest->data;
                u8 *u_plane  = dest->data + y_size;
                u8 *v_plane  = dest->data + y_size + c_size;

                const JDIMENSION lines_per_iMCU_row_y  = (JDIMENSION)(ciY->v_samp_factor * DCTSIZE);   // 16
                const JDIMENSION lines_per_iMCU_row_c  = (JDIMENSION)(ciCb->v_samp_factor * DCTSIZE);  // 8

                // 行指针数组（最大 16 / 8 行）
                JSAMPROW y_rows[16];
                JSAMPROW cb_rows[8];
                JSAMPROW cr_rows[8];
                JSAMPARRAY planes[3] = { y_rows, cb_rows, cr_rows };

                while (jpeg.output_scanline < jpeg.output_height) {
                    JDIMENSION y_base = jpeg.output_scanline;
                    // 准备 Y 行指针
                    for (JDIMENSION i = 0; i < lines_per_iMCU_row_y; ++i) {
                        JDIMENSION y_idx = y_base + i;
                        if (y_idx >= jpeg.output_height) { y_rows[i] = y_plane + (dest->height - 1) * dest->stride; }
                        else { y_rows[i] = y_plane + y_idx * dest->stride; }
                    }
                    // 对应的 Cb/Cr 行基准（按 2:1 垂直采样）
                    JDIMENSION c_base = y_base / 2;
                    for (JDIMENSION i = 0; i < lines_per_iMCU_row_c; ++i) {
                        JDIMENSION c_idx = c_base + i;
                        if (c_idx >= uv_h) {
                            cb_rows[i] = u_plane + (uv_h - 1) * uv_w;
                            cr_rows[i] = v_plane + (uv_h - 1) * uv_w;
                        } else {
                            cb_rows[i] = u_plane + c_idx * uv_w;
                            cr_rows[i] = v_plane + c_idx * uv_w;
                        }
                    }
                    (void)jpeg_read_raw_data(&jpeg, planes, lines_per_iMCU_row_y);
                }

                jpeg_finish_decompress(&jpeg);
                yuv420_ok = true;
            }
        }
    }

    if (!yuv420_ok) {
        // 回退：输出 RGB24（与之前逻辑一致）
        jpeg.out_color_space = JCS_RGB;
        jpeg_start_decompress(&jpeg);

        US_FRAME_COPY_META(src, dest);
        dest->format = V4L2_PIX_FMT_RGB24;
        dest->width = jpeg.output_width;
        dest->height = jpeg.output_height;
        dest->stride = jpeg.output_width * jpeg.output_components;
        dest->used = 0;

        if (decode) {
            JSAMPARRAY scanlines = (*jpeg.mem->alloc_sarray)((j_common_ptr) &jpeg, JPOOL_IMAGE, dest->stride, 1);
            us_frame_realloc_data(dest, ((dest->width * dest->height) << 1) * 2);
            while (jpeg.output_scanline < jpeg.output_height) {
                jpeg_read_scanlines(&jpeg, scanlines, 1);
                us_frame_append_data(dest, scanlines[0], dest->stride);
            }
            jpeg_finish_decompress(&jpeg);
        }
    }

done:
	jpeg_destroy_decompress(&jpeg);
	return retval;
}

static void _jpeg_error_handler(j_common_ptr jpeg) {
	_jpeg_error_manager_s *jpeg_error = (_jpeg_error_manager_s*)jpeg->err;
	char msg[JMSG_LENGTH_MAX];

	(*jpeg_error->mgr.format_message)(jpeg, msg);
	US_LOG_ERROR("Can't decompress JPEG: %s", msg);
	longjmp(jpeg_error->jmp, -1);
}
