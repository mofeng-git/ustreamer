#include "mpp_encoder.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

// uStreamer内部头文件
#include "../../../libs/logging.h"
#include "../../../libs/tools.h"
#include "../../../libs/frame.h"

#define US_MPP_TRANSCODER_LOG_PREFIX "[MPP-TRANSCODER] "
#define US_MPP_TRANSCODER_LOG_INFO(fmt, ...) US_LOG_INFO(US_MPP_TRANSCODER_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_TRANSCODER_LOG_ERROR(fmt, ...) US_LOG_ERROR(US_MPP_TRANSCODER_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_TRANSCODER_LOG_DEBUG(fmt, ...) US_LOG_DEBUG(US_MPP_TRANSCODER_LOG_PREFIX fmt, ##__VA_ARGS__)

// 前向声明（这些函数在mmp_encoder.c中实现）
uint64_t _us_mpp_get_time_us(void);

// ==== 一体化编解码API实现 ====

us_mpp_error_e us_mpp_transcoder_create(us_mpp_transcoder_s **transcoder,
                                       uint32_t max_width, uint32_t max_height,
                                       uint32_t bitrate_kbps, uint32_t gop_size,
                                       uint32_t fps_num, uint32_t fps_den) {
    if (!transcoder) {
        US_MPP_TRANSCODER_LOG_ERROR("Transcoder pointer is NULL");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (max_width == 0 || max_height == 0 || bitrate_kbps == 0 || fps_num == 0 || fps_den == 0) {
        US_MPP_TRANSCODER_LOG_ERROR("Invalid parameters: %ux%u, %u kbps, %u/%u fps", 
                                   max_width, max_height, bitrate_kbps, fps_num, fps_den);
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    *transcoder = NULL;
    
    // 分配transcoder结构
    us_mpp_transcoder_s *tc = calloc(1, sizeof(us_mpp_transcoder_s));
    if (!tc) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to allocate transcoder structure");
        return US_MPP_ERROR_MEMORY;
    }
    
    // 初始化互斥锁
    if (pthread_mutex_init(&tc->mutex, NULL) != 0) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to initialize transcoder mutex");
        free(tc);
        return US_MPP_ERROR_MEMORY;
    }
    
    // 创建MJPEG解码器（可选，延迟创建）
    tc->decoder = NULL;
    
    // 创建H264编码器  
    us_mpp_error_e err = us_mpp_h264_encoder_create(&tc->encoder, max_width, max_height, 
                                                   bitrate_kbps, gop_size, fps_num, fps_den);
    if (err != US_MPP_OK) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to create H264 encoder: %s", us_mpp_error_string(err));
        pthread_mutex_destroy(&tc->mutex);
        free(tc);
        return err;
    }
    
    // 分配中间NV12缓冲区
    tc->nv12_buffer = us_frame_init();
    if (!tc->nv12_buffer) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to allocate NV12 buffer");
        us_mpp_processor_destroy(tc->encoder);
        pthread_mutex_destroy(&tc->mutex);
        free(tc);
        return US_MPP_ERROR_MEMORY;
    }
    
    // 分配格式转换缓冲区
    tc->conversion_buffer = us_frame_init();
    if (!tc->conversion_buffer) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to allocate conversion buffer");
        us_frame_destroy(tc->nv12_buffer);
        us_mpp_processor_destroy(tc->encoder);
        pthread_mutex_destroy(&tc->mutex);
        free(tc);
        return US_MPP_ERROR_MEMORY;
    }
    
    // 初始化格式转换信息
    tc->current_input_format = 0;
    tc->needs_format_conversion = false;
    memset(&tc->format_info, 0, sizeof(tc->format_info));
    
    tc->initialized = true;
    *transcoder = tc;
    
    US_MPP_TRANSCODER_LOG_INFO("Multi-format MPP transcoder created: %ux%u, %u kbps, GOP %u, %u/%u fps",
                              max_width, max_height, bitrate_kbps, gop_size, fps_num, fps_den);
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_transcoder_process(us_mpp_transcoder_s *transcoder,
                                        const us_frame_s *input_frame,
                                        us_frame_s *h264_frame,
                                        bool force_key) {
    if (!transcoder || !input_frame || !h264_frame) {
        US_MPP_TRANSCODER_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!transcoder->initialized) {
        US_MPP_TRANSCODER_LOG_ERROR("Transcoder not initialized");
        return US_MPP_ERROR_NOT_INITIALIZED;
    }
    
    // 检查输入格式是否支持
    if (!us_mpp_is_format_supported(input_frame->format)) {
        US_MPP_TRANSCODER_LOG_ERROR("Unsupported input format: %u", input_frame->format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    us_mpp_error_e result = US_MPP_OK;
    
    pthread_mutex_lock(&transcoder->mutex);
    
    // 检查是否需要更新格式转换信息（MJPEG/JPEG通过硬件解码器处理，不需要格式转换）
    if (transcoder->current_input_format != input_frame->format) {
        if (input_frame->format == V4L2_PIX_FMT_MJPEG || input_frame->format == V4L2_PIX_FMT_JPEG) {
            // MJPEG/JPEG通过硬件解码器处理，不需要格式转换
            transcoder->current_input_format = input_frame->format;
            transcoder->needs_format_conversion = false;
            transcoder->format_info.needs_conversion = false;
            transcoder->format_info.conversion_type = US_MPP_FORMAT_CONVERSION_NONE;
        } else {
            // 其他格式需要格式转换
            us_mpp_error_e format_err = us_mpp_get_format_conversion_info(input_frame->format, 
                                                                         V4L2_PIX_FMT_NV12,
                                                                         &transcoder->format_info);
            if (format_err != US_MPP_OK) {
                US_MPP_TRANSCODER_LOG_ERROR("Failed to get format conversion info: %s", us_mpp_error_string(format_err));
                result = format_err;
                goto cleanup;
            }
            
            transcoder->current_input_format = input_frame->format;
            transcoder->needs_format_conversion = transcoder->format_info.needs_conversion;
        }
        
        US_MPP_TRANSCODER_LOG_INFO("Input format changed to: %u, conversion needed: %s", 
                                  input_frame->format, transcoder->needs_format_conversion ? "yes" : "no");
    }
    
    us_frame_s *nv12_input = NULL;
    
    // 步骤1: 处理输入格式转换
    if (input_frame->format == V4L2_PIX_FMT_MJPEG || input_frame->format == V4L2_PIX_FMT_JPEG) {
        // MJPEG解码路径
        if (!transcoder->decoder) {
            // 延迟创建MJPEG解码器
            us_mpp_error_e decode_err = us_mpp_mjpeg_decoder_create(&transcoder->decoder, 
                                                                   input_frame->width, 
                                                                   input_frame->height);
            if (decode_err != US_MPP_OK) {
                US_MPP_TRANSCODER_LOG_ERROR("Failed to create MJPEG decoder: %s", us_mpp_error_string(decode_err));
                result = decode_err;
                goto cleanup;
            }
        }
        
        us_mpp_error_e decode_err = us_mpp_mjpeg_decoder_decode(transcoder->decoder, 
                                                               input_frame, 
                                                               transcoder->nv12_buffer);
        if (decode_err != US_MPP_OK) {
            US_MPP_TRANSCODER_LOG_ERROR("MJPEG decode failed: %s", us_mpp_error_string(decode_err));
            result = decode_err;
            goto cleanup;
        }
        nv12_input = transcoder->nv12_buffer;
        
    } else if (input_frame->format == V4L2_PIX_FMT_NV12) {
        // NV12直接传递路径（零拷贝优化）
        nv12_input = (us_frame_s *)input_frame;
        
    } else {
        // 其他格式需要CPU转换
        us_mpp_error_e convert_err = us_mpp_convert_format(input_frame, 
                                                          transcoder->conversion_buffer,
                                                          V4L2_PIX_FMT_NV12);
        if (convert_err != US_MPP_OK) {
            US_MPP_TRANSCODER_LOG_ERROR("Format conversion failed: %s", us_mpp_error_string(convert_err));
            result = convert_err;
            goto cleanup;
        }
        nv12_input = transcoder->conversion_buffer;
    }
    
    // 步骤2: NV12编码到H264
    us_mpp_error_e encode_err = us_mpp_h264_encoder_encode(transcoder->encoder,
                                                          nv12_input,
                                                          h264_frame,
                                                          force_key);
    if (encode_err != US_MPP_OK) {
        US_MPP_TRANSCODER_LOG_ERROR("H264 encode failed: %s", us_mpp_error_string(encode_err));
        result = encode_err;
        goto cleanup;
    }
    
cleanup:
    pthread_mutex_unlock(&transcoder->mutex);
    return result;
}

void us_mpp_transcoder_destroy(us_mpp_transcoder_s *transcoder) {
    if (!transcoder) return;
    
    US_MPP_TRANSCODER_LOG_INFO("Destroying MPP transcoder");
    
    pthread_mutex_lock(&transcoder->mutex);
    
    // 销毁组件
    if (transcoder->decoder) {
        us_mpp_processor_destroy(transcoder->decoder);
        transcoder->decoder = NULL;
    }
    
    if (transcoder->encoder) {
        us_mpp_processor_destroy(transcoder->encoder);
        transcoder->encoder = NULL;
    }
    
    if (transcoder->nv12_buffer) {
        us_frame_destroy(transcoder->nv12_buffer);
        transcoder->nv12_buffer = NULL;
    }
    
    if (transcoder->conversion_buffer) {
        us_frame_destroy(transcoder->conversion_buffer);
        transcoder->conversion_buffer = NULL;
    }
    
    transcoder->initialized = false;
    
    // 打印最终统计信息
    US_MPP_TRANSCODER_LOG_INFO("Final stats - Processed: %lu, Decoded: %lu, Encoded: %lu, Errors: %lu, Avg time: %.2f ms",
                              transcoder->combined_stats.frames_processed,
                              transcoder->combined_stats.frames_decoded, 
                              transcoder->combined_stats.frames_encoded,
                              transcoder->combined_stats.processing_errors,
                              transcoder->combined_stats.avg_processing_time_ms);
    
    pthread_mutex_unlock(&transcoder->mutex);
    pthread_mutex_destroy(&transcoder->mutex);
    
    free(transcoder);
}

us_mpp_error_e us_mpp_transcoder_get_stats(us_mpp_transcoder_s *transcoder, us_mpp_stats_s *stats) {
    if (!transcoder || !stats) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!transcoder->initialized) {
        return US_MPP_ERROR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&transcoder->mutex);
    *stats = transcoder->combined_stats;
    pthread_mutex_unlock(&transcoder->mutex);
    
    return US_MPP_OK;
}
