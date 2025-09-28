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
    
    // 创建MJPEG解码器
    us_mpp_error_e err = us_mpp_mjpeg_decoder_create(&tc->decoder, max_width, max_height);
    if (err != US_MPP_OK) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to create MJPEG decoder: %s", us_mpp_error_string(err));
        pthread_mutex_destroy(&tc->mutex);
        free(tc);
        return err;
    }
    
    // 创建H264编码器  
    err = us_mpp_h264_encoder_create(&tc->encoder, max_width, max_height, 
                                    bitrate_kbps, gop_size, fps_num, fps_den);
    if (err != US_MPP_OK) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to create H264 encoder: %s", us_mpp_error_string(err));
        us_mpp_processor_destroy(tc->decoder);
        pthread_mutex_destroy(&tc->mutex);
        free(tc);
        return err;
    }
    
    // 分配中间NV12缓冲区
    tc->nv12_buffer = us_frame_init();
    if (!tc->nv12_buffer) {
        US_MPP_TRANSCODER_LOG_ERROR("Failed to allocate NV12 buffer");
        us_mpp_processor_destroy(tc->encoder);
        us_mpp_processor_destroy(tc->decoder);
        pthread_mutex_destroy(&tc->mutex);
        free(tc);
        return US_MPP_ERROR_MEMORY;
    }
    
    tc->initialized = true;
    *transcoder = tc;
    
    US_MPP_TRANSCODER_LOG_INFO("MPP transcoder created successfully: %ux%u, %u kbps, GOP %u, %u/%u fps",
                              max_width, max_height, bitrate_kbps, gop_size, fps_num, fps_den);
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_transcoder_process(us_mpp_transcoder_s *transcoder,
                                        const us_frame_s *mjpeg_frame,
                                        us_frame_s *h264_frame,
                                        bool force_key) {
    if (!transcoder || !mjpeg_frame || !h264_frame) {
        US_MPP_TRANSCODER_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!transcoder->initialized) {
        US_MPP_TRANSCODER_LOG_ERROR("Transcoder not initialized");
        return US_MPP_ERROR_NOT_INITIALIZED;
    }
    
    uint64_t start_time = _us_mpp_get_time_us();
    us_mpp_error_e result = US_MPP_OK;
    
    pthread_mutex_lock(&transcoder->mutex);
    
    // 步骤1: MJPEG解码到NV12
    US_MPP_TRANSCODER_LOG_DEBUG("Decoding MJPEG frame: %ux%u, %zu bytes", 
                               mjpeg_frame->width, mjpeg_frame->height, mjpeg_frame->used);
    
    us_mpp_error_e decode_err = us_mpp_mjpeg_decoder_decode(transcoder->decoder, 
                                                           mjpeg_frame, 
                                                           transcoder->nv12_buffer);
    if (decode_err != US_MPP_OK) {
        US_MPP_TRANSCODER_LOG_ERROR("MJPEG decode failed: %s", us_mpp_error_string(decode_err));
        result = decode_err;
        goto cleanup;
    }
    
    // 步骤2: NV12编码到H264
    US_MPP_TRANSCODER_LOG_DEBUG("Encoding NV12 to H264: %ux%u, %zu bytes", 
                               transcoder->nv12_buffer->width, transcoder->nv12_buffer->height, 
                               transcoder->nv12_buffer->used);
    
    us_mpp_error_e encode_err = us_mpp_h264_encoder_encode(transcoder->encoder,
                                                          transcoder->nv12_buffer,
                                                          h264_frame,
                                                          force_key);
    if (encode_err != US_MPP_OK) {
        US_MPP_TRANSCODER_LOG_ERROR("H264 encode failed: %s", us_mpp_error_string(encode_err));
        result = encode_err;
        goto cleanup;
    }
    
    // 更新合并统计信息
    transcoder->combined_stats.frames_processed++;
    if (result == US_MPP_OK) {
        transcoder->combined_stats.frames_decoded++;
        transcoder->combined_stats.frames_encoded++;
        transcoder->combined_stats.bytes_input += mjpeg_frame->used;
        transcoder->combined_stats.bytes_output += h264_frame->used;
    } else {
        transcoder->combined_stats.processing_errors++;
    }
    
cleanup:
    pthread_mutex_unlock(&transcoder->mutex);
    
    uint64_t end_time = _us_mpp_get_time_us();
    uint64_t process_time = end_time - start_time;
    
    if (result == US_MPP_OK) {
        transcoder->combined_stats.total_processing_time_ms += process_time / 1000.0;
        transcoder->combined_stats.avg_processing_time_ms = 
            transcoder->combined_stats.total_processing_time_ms / transcoder->combined_stats.frames_processed;
        
        US_MPP_TRANSCODER_LOG_DEBUG("Transcode success: %ux%u MJPEG (%zu B) -> %ux%u H264 (%zu B) in %.2f ms %s",
                                   mjpeg_frame->width, mjpeg_frame->height, mjpeg_frame->used,
                                   h264_frame->width, h264_frame->height, h264_frame->used,
                                   process_time / 1000.0, force_key ? "[KEY]" : "");
    } else {
        US_MPP_TRANSCODER_LOG_ERROR("Transcode failed: %s (%.2f ms)",
                                   us_mpp_error_string(result), process_time / 1000.0);
    }
    
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
