/*
 * MPP格式转换器实现
 * 支持多种输入格式到NV12的CPU转换
 * 使用libyuv和libjpeg-turbo优化性能
 */

#include "mpp_encoder.h"
#include <string.h>
#include <stdlib.h>

// 包含优化库
#include <libyuv.h>

// 保留错误日志，删除信息日志
#define US_MPP_FORMAT_LOG_ERROR(fmt, ...) \
    fprintf(stderr, "[MPP-FORMAT] ERROR: " fmt "\n", ##__VA_ARGS__)


// 计算帧大小
static size_t _us_mpp_calc_frame_size_by_format(uint32_t width, uint32_t height, uint32_t format) {
    switch (format) {
        case V4L2_PIX_FMT_RGB24:
        case V4L2_PIX_FMT_BGR24:
            return width * height * 3;
        case V4L2_PIX_FMT_YUYV:
            return width * height * 2;
        case V4L2_PIX_FMT_NV12:
            return width * height * 3 / 2;
        case V4L2_PIX_FMT_NV16:
            return width * height * 2;
        case V4L2_PIX_FMT_YUV420:
            return width * height * 3 / 2;
        default:
            return 0;
    }
}

// RGB24/BGR24 到 NV12 转换（使用libyuv优化）
us_mpp_error_e _us_mpp_convert_rgb_to_nv12(const us_frame_s *rgb_frame, us_frame_s *nv12_frame) {
    if (!rgb_frame || !nv12_frame) {
        US_MPP_FORMAT_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (rgb_frame->format != V4L2_PIX_FMT_RGB24 && rgb_frame->format != V4L2_PIX_FMT_BGR24) {
        US_MPP_FORMAT_LOG_ERROR("Unsupported input format: %u", rgb_frame->format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    uint32_t width = rgb_frame->width;
    uint32_t height = rgb_frame->height;
    size_t nv12_size = _us_mpp_calc_frame_size_by_format(width, height, V4L2_PIX_FMT_NV12);
    
    // 确保输出缓冲区足够大
    if (nv12_frame->allocated < nv12_size) {
        // 重新分配缓冲区
        us_frame_realloc_data(nv12_frame, nv12_size);
        if (!nv12_frame->data) {
            US_MPP_FORMAT_LOG_ERROR("Failed to reallocate output buffer to %zu bytes", nv12_size);
            return US_MPP_ERROR_MEMORY;
        }
    }
    
    uint8_t *rgb_data = (uint8_t *)rgb_frame->data;
    uint8_t *nv12_data = (uint8_t *)nv12_frame->data;
    
    // Y平面
    uint8_t *y_plane = nv12_data;
    // UV平面
    uint8_t *uv_plane = nv12_data + width * height;
    
    // libyuv没有直接的RGB24/BGR24转NV12函数，使用软件实现
    bool is_bgr = (rgb_frame->format == V4L2_PIX_FMT_BGR24);
    
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            uint32_t rgb_idx = (y * width + x) * 3;
            uint32_t y_idx = y * width + x;
            
            uint8_t r, g, b;
            if (is_bgr) {
                b = rgb_data[rgb_idx];
                g = rgb_data[rgb_idx + 1];
                r = rgb_data[rgb_idx + 2];
            } else {
                r = rgb_data[rgb_idx];
                g = rgb_data[rgb_idx + 1];
                b = rgb_data[rgb_idx + 2];
            }
            
            // RGB to YUV conversion (ITU-R BT.601)
            int32_t y_val = (int32_t)(0.299 * r + 0.587 * g + 0.114 * b);
            y_plane[y_idx] = (uint8_t)(y_val < 0 ? 0 : (y_val > 255 ? 255 : y_val));
        }
    }
    
    // UV平面处理（每2x2像素块计算一个UV值）
    for (uint32_t y = 0; y < height; y += 2) {
        for (uint32_t x = 0; x < width; x += 2) {
            uint32_t uv_idx = (y / 2) * (width / 2) + (x / 2);
            
            // 计算2x2块的UV值
            int32_t u_sum = 0, v_sum = 0;
            for (int dy = 0; dy < 2 && y + dy < height; dy++) {
                for (int dx = 0; dx < 2 && x + dx < width; dx++) {
                    uint32_t rgb_idx = ((y + dy) * width + (x + dx)) * 3;
                    
                    uint8_t r, g, b;
                    if (is_bgr) {
                        b = rgb_data[rgb_idx];
                        g = rgb_data[rgb_idx + 1];
                        r = rgb_data[rgb_idx + 2];
                    } else {
                        r = rgb_data[rgb_idx];
                        g = rgb_data[rgb_idx + 1];
                        b = rgb_data[rgb_idx + 2];
                    }
                    
                    // RGB to YUV conversion
                    int32_t u_val = (int32_t)(-0.147 * r - 0.289 * g + 0.436 * b + 128);
                    int32_t v_val = (int32_t)(0.615 * r - 0.515 * g - 0.100 * b + 128);
                    
                    u_sum += u_val;
                    v_sum += v_val;
                }
            }
            
            uv_plane[uv_idx * 2] = (uint8_t)((u_sum / 4) < 0 ? 0 : ((u_sum / 4) > 255 ? 255 : (u_sum / 4)));
            uv_plane[uv_idx * 2 + 1] = (uint8_t)((v_sum / 4) < 0 ? 0 : ((v_sum / 4) > 255 ? 255 : (v_sum / 4)));
        }
    }

    // 设置输出帧信息
    nv12_frame->width = width;
    nv12_frame->height = height;
    nv12_frame->format = V4L2_PIX_FMT_NV12;
    nv12_frame->used = nv12_size;
    
    return US_MPP_OK;
}

// YUYV 到 NV12 转换（使用libyuv优化）
us_mpp_error_e _us_mpp_convert_yuyv_to_nv12(const us_frame_s *yuyv_frame, us_frame_s *nv12_frame) {
    if (!yuyv_frame || !nv12_frame) {
        US_MPP_FORMAT_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (yuyv_frame->format != V4L2_PIX_FMT_YUYV) {
        US_MPP_FORMAT_LOG_ERROR("Unsupported input format: %u", yuyv_frame->format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    uint32_t width = yuyv_frame->width;
    uint32_t height = yuyv_frame->height;
    size_t nv12_size = _us_mpp_calc_frame_size_by_format(width, height, V4L2_PIX_FMT_NV12);
    
    if (nv12_frame->allocated < nv12_size) {
        // 重新分配缓冲区
        us_frame_realloc_data(nv12_frame, nv12_size);
        if (!nv12_frame->data) {
            US_MPP_FORMAT_LOG_ERROR("Failed to reallocate output buffer to %zu bytes", nv12_size);
            return US_MPP_ERROR_MEMORY;
        }
    }
    
    uint8_t *yuyv_data = (uint8_t *)yuyv_frame->data;
    uint8_t *nv12_data = (uint8_t *)nv12_frame->data;
    
    // Y平面
    uint8_t *y_plane = nv12_data;
    // UV平面
    uint8_t *uv_plane = nv12_data + width * height;
    
    // 使用libyuv进行高效转换
    int ret = YUY2ToNV12(yuyv_data, width * 2,
                        y_plane, width,
                        uv_plane, width,
                        width, height);
    
    if (ret != 0) {
        US_MPP_FORMAT_LOG_ERROR("libyuv YUYV->NV12 conversion failed: %d", ret);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    
    // 设置输出帧信息
    nv12_frame->width = width;
    nv12_frame->height = height;
    nv12_frame->format = V4L2_PIX_FMT_NV12;
    nv12_frame->used = nv12_size;
    
    return US_MPP_OK;
}

// YUV420 到 NV12 转换（使用libyuv优化）
us_mpp_error_e _us_mpp_convert_yuv420_to_nv12(const us_frame_s *yuv420_frame, us_frame_s *nv12_frame) {
    if (!yuv420_frame || !nv12_frame) {
        US_MPP_FORMAT_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (yuv420_frame->format != V4L2_PIX_FMT_YUV420) {
        US_MPP_FORMAT_LOG_ERROR("Unsupported input format: %u", yuv420_frame->format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    uint32_t width = yuv420_frame->width;
    uint32_t height = yuv420_frame->height;
    size_t nv12_size = _us_mpp_calc_frame_size_by_format(width, height, V4L2_PIX_FMT_NV12);
    
    if (nv12_frame->allocated < nv12_size) {
        // 重新分配缓冲区
        us_frame_realloc_data(nv12_frame, nv12_size);
        if (!nv12_frame->data) {
            US_MPP_FORMAT_LOG_ERROR("Failed to reallocate output buffer to %zu bytes", nv12_size);
            return US_MPP_ERROR_MEMORY;
        }
    }
    
    uint8_t *yuv420_data = (uint8_t *)yuv420_frame->data;
    uint8_t *nv12_data = (uint8_t *)nv12_frame->data;
    
    // Y平面
    uint8_t *y_plane = nv12_data;
    // UV平面
    uint8_t *uv_plane = nv12_data + width * height;
    
    // 使用libyuv进行高效转换
    uint8_t *u_plane = yuv420_data + width * height;
    uint8_t *v_plane = u_plane + (width * height) / 4;
    
    int ret = I420ToNV12(y_plane, width,
                        u_plane, width / 2,
                        v_plane, width / 2,
                        y_plane, width,
                        uv_plane, width,
                        width, height);
    
    if (ret != 0) {
        US_MPP_FORMAT_LOG_ERROR("libyuv YUV420->NV12 conversion failed: %d", ret);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    
    // 设置输出帧信息
    nv12_frame->width = width;
    nv12_frame->height = height;
    nv12_frame->format = V4L2_PIX_FMT_NV12;
    nv12_frame->used = nv12_size;
    
    return US_MPP_OK;
}

// NV16 到 NV12 转换
us_mpp_error_e _us_mpp_convert_nv16_to_nv12(const us_frame_s *nv16_frame, us_frame_s *nv12_frame) {
    if (!nv16_frame || !nv12_frame) {
        US_MPP_FORMAT_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (nv16_frame->format != V4L2_PIX_FMT_NV16) {
        US_MPP_FORMAT_LOG_ERROR("Unsupported input format: %u", nv16_frame->format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    uint32_t width = nv16_frame->width;
    uint32_t height = nv16_frame->height;
    size_t nv12_size = _us_mpp_calc_frame_size_by_format(width, height, V4L2_PIX_FMT_NV12);
    
    if (nv12_frame->allocated < nv12_size) {
        // 重新分配缓冲区
        us_frame_realloc_data(nv12_frame, nv12_size);
        if (!nv12_frame->data) {
            US_MPP_FORMAT_LOG_ERROR("Failed to reallocate output buffer to %zu bytes", nv12_size);
            return US_MPP_ERROR_MEMORY;
        }
    }
    
    uint8_t *nv16_data = (uint8_t *)nv16_frame->data;
    uint8_t *nv12_data = (uint8_t *)nv12_frame->data;
    
    // Y平面
    uint8_t *y_plane = nv12_data;
    // UV平面
    uint8_t *uv_plane = nv12_data + width * height;
    
    // 复制Y分量
    memcpy(y_plane, nv16_data, width * height);
    
    // 处理UV分量（NV16是4:2:2，NV12是4:2:0，需要下采样）
    uint8_t *nv16_uv = nv16_data + width * height;
    
    for (uint32_t y = 0; y < height; y += 2) {
        for (uint32_t x = 0; x < width; x += 2) {
            uint32_t nv16_uv_idx = y * width + x;
            uint32_t nv12_uv_idx = (y / 2) * (width / 2) + (x / 2);
            
            // 取第一个像素的UV值
            uv_plane[nv12_uv_idx * 2] = nv16_uv[nv16_uv_idx * 2];     // U
            uv_plane[nv12_uv_idx * 2 + 1] = nv16_uv[nv16_uv_idx * 2 + 1]; // V
        }
    }
    
    // 设置输出帧信息
    nv12_frame->width = width;
    nv12_frame->height = height;
    nv12_frame->format = V4L2_PIX_FMT_NV12;
    nv12_frame->used = nv12_size;
    
    return US_MPP_OK;
}

// 通用格式转换函数
us_mpp_error_e us_mpp_convert_format(const us_frame_s *input_frame, 
                                    us_frame_s *output_frame,
                                    uint32_t target_format) {
    if (!input_frame || !output_frame) {
        US_MPP_FORMAT_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (target_format != V4L2_PIX_FMT_NV12) {
        US_MPP_FORMAT_LOG_ERROR("Only NV12 target format is supported");
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    // 如果输入已经是NV12，直接复制
    if (input_frame->format == V4L2_PIX_FMT_NV12) {
        size_t nv12_size = _us_mpp_calc_frame_size_by_format(input_frame->width, input_frame->height, V4L2_PIX_FMT_NV12);
        if (output_frame->allocated < nv12_size) {
            // 重新分配缓冲区
            us_frame_realloc_data(output_frame, nv12_size);
            if (!output_frame->data) {
                US_MPP_FORMAT_LOG_ERROR("Failed to reallocate output buffer to %zu bytes", nv12_size);
                return US_MPP_ERROR_MEMORY;
            }
        }
        
        memcpy(output_frame->data, input_frame->data, nv12_size);
        output_frame->width = input_frame->width;
        output_frame->height = input_frame->height;
        output_frame->format = V4L2_PIX_FMT_NV12;
        output_frame->used = nv12_size;
        return US_MPP_OK;
    }
    
    // 根据输入格式选择转换函数
    switch (input_frame->format) {
        case V4L2_PIX_FMT_RGB24:
        case V4L2_PIX_FMT_BGR24:
            return _us_mpp_convert_rgb_to_nv12(input_frame, output_frame);
        case V4L2_PIX_FMT_YUYV:
            return _us_mpp_convert_yuyv_to_nv12(input_frame, output_frame);
        case V4L2_PIX_FMT_YUV420:
            return _us_mpp_convert_yuv420_to_nv12(input_frame, output_frame);
        case V4L2_PIX_FMT_NV16:
            return _us_mpp_convert_nv16_to_nv12(input_frame, output_frame);
        default:
            US_MPP_FORMAT_LOG_ERROR("Unsupported input format: %u", input_frame->format);
            return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
}

// 获取格式转换信息
us_mpp_error_e us_mpp_get_format_conversion_info(uint32_t input_format, 
                                                uint32_t output_format,
                                                us_mpp_format_info_s *info) {
    if (!info) {
        US_MPP_FORMAT_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    info->input_format = input_format;
    info->output_format = output_format;
    
    // 检查是否需要转换
    if (input_format == output_format) {
        info->needs_conversion = false;
        info->conversion_type = US_MPP_FORMAT_CONVERSION_NONE;
        return US_MPP_OK;
    }
    
    // 检查是否支持转换
    bool supported = false;
    switch (input_format) {
        case V4L2_PIX_FMT_RGB24:
        case V4L2_PIX_FMT_BGR24:
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_NV16:
        case V4L2_PIX_FMT_NV12:
            supported = (output_format == V4L2_PIX_FMT_NV12);
            break;
        default:
            supported = false;
            break;
    }
    
    if (!supported) {
        US_MPP_FORMAT_LOG_ERROR("Unsupported format conversion: %u -> %u", input_format, output_format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    info->needs_conversion = true;
    info->conversion_type = US_MPP_FORMAT_CONVERSION_CPU;
    
    return US_MPP_OK;
}

// 检查格式是否支持
bool us_mpp_is_format_supported(uint32_t format) {
    switch (format) {
        case V4L2_PIX_FMT_MJPEG:
        case V4L2_PIX_FMT_JPEG:
        case V4L2_PIX_FMT_RGB24:
        case V4L2_PIX_FMT_BGR24:
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV16:
        case V4L2_PIX_FMT_YUV420:
            return true;
        default:
            return false;
    }
}

