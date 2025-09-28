#include "mpp_encoder.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <linux/videodev2.h>

// MPP头文件
#include "rk_mpi.h"
#include "mpp_frame.h"
#include "mpp_packet.h"

// uStreamer内部头文件
#include "../../../libs/logging.h"
#include "../../../libs/tools.h"
#include "../../../libs/frame.h"

#define US_MPP_DECODER_BUFFER_COUNT 8
#define US_MPP_ENCODER_BUFFER_COUNT 8
#define US_MPP_MAX_BUFFER_SIZE (1920 * 1080 * 3 / 2)  // 最大NV12帧大小
#define US_MPP_TIMEOUT_MS 100
#define US_MPP_MAX_CONSECUTIVE_ERRORS 10

// ==== 辅助宏定义 ====
#define US_MPP_LOG_PREFIX "[MPP-ENC] "
#define US_MPP_LOG_INFO(fmt, ...) US_LOG_INFO(US_MPP_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_LOG_ERROR(fmt, ...) US_LOG_ERROR(US_MPP_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_LOG_DEBUG(fmt, ...) US_LOG_DEBUG(US_MPP_LOG_PREFIX fmt, ##__VA_ARGS__)

#define US_MPP_CHECK_NULL(ptr, name) \
    do { \
        if (!(ptr)) { \
            US_MPP_LOG_ERROR("%s is NULL", name); \
            return US_MPP_ERROR_INVALID_PARAM; \
        } \
    } while (0)

#define US_MPP_CHECK_INIT(proc, name) \
    do { \
        if (!(proc) || !atomic_load(&(proc)->initialized)) { \
            US_MPP_LOG_ERROR("%s not initialized", name); \
            return US_MPP_ERROR_NOT_INITIALIZED; \
        } \
    } while (0)

// ==== 内部辅助函数 ====

uint64_t _us_mpp_get_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000ULL + tv.tv_usec;
}

// 按照MPP测试标准计算frame大小（关键优化）
uint32_t _us_mpp_calc_frame_size(uint32_t width, uint32_t height, MppFrameFormat fmt) {
    uint32_t hor_stride = (width + 15) & ~15;   // 16字节对齐
    uint32_t ver_stride = (height + 15) & ~15;  // 16字节对齐
    uint32_t frame_size = 0;
    
    // 按照MPP测试的精确计算方法（参考mpi_enc_test.c line 337-372）
    switch (fmt & MPP_FRAME_FMT_MASK) {
    case MPP_FMT_YUV420SP:
    case MPP_FMT_YUV420P: {
        // NV12/YUV420P: 使用64字节对齐，乘以1.5
        frame_size = ((hor_stride + 63) & ~63) * ((ver_stride + 63) & ~63) * 3 / 2;
    } break;
    
    case MPP_FMT_YUV422_YUYV:
    case MPP_FMT_YUV422_YVYU:
    case MPP_FMT_YUV422_UYVY:
    case MPP_FMT_YUV422_VYUY:
    case MPP_FMT_YUV422P:
    case MPP_FMT_YUV422SP: {
        // YUV422: 使用64字节对齐，乘以2
        frame_size = ((hor_stride + 63) & ~63) * ((ver_stride + 63) & ~63) * 2;
    } break;
    
    case MPP_FMT_RGB888:
    case MPP_FMT_BGR888: {
        // RGB888: 3字节每像素
        frame_size = ((hor_stride + 63) & ~63) * ((ver_stride + 63) & ~63) * 3;
    } break;
    
    case MPP_FMT_ARGB8888:
    case MPP_FMT_ABGR8888:
    case MPP_FMT_BGRA8888:
    case MPP_FMT_RGBA8888: {
        // RGBA: 4字节每像素
        frame_size = ((hor_stride + 63) & ~63) * ((ver_stride + 63) & ~63) * 4;
    } break;
    
    default: {
        // 默认按4字节每像素分配（安全的估计）
        frame_size = ((hor_stride + 63) & ~63) * ((ver_stride + 63) & ~63) * 4;
    } break;
    }
    
    return frame_size;
}

static const char* _us_mpp_error_to_string(us_mpp_error_e error) {
    switch (error) {
        case US_MPP_OK: return "OK";
        case US_MPP_ERROR_INVALID_PARAM: return "Invalid parameter";
        case US_MPP_ERROR_MEMORY: return "Memory allocation failed";
        case US_MPP_ERROR_INIT: return "Initialization failed";
        case US_MPP_ERROR_ENCODE: return "Encoding failed";
        case US_MPP_ERROR_DECODE: return "Decoding failed";
        case US_MPP_ERROR_FORMAT_UNSUPPORTED: return "Format not supported";
        case US_MPP_ERROR_DEVICE_NOT_FOUND: return "Device not found";
        case US_MPP_ERROR_DEVICE_BUSY: return "Device busy";
        case US_MPP_ERROR_HARDWARE_FAILURE: return "Hardware failure";
        case US_MPP_ERROR_NOT_INITIALIZED: return "Not initialized";
        case US_MPP_ERROR_BUFFER_OVERFLOW: return "Buffer overflow";
        case US_MPP_ERROR_TIMEOUT: return "Operation timeout";
        case US_MPP_ERROR_INFO_CHANGE: return "Info change event";
        case US_MPP_ERROR_EOS: return "End of stream";
        default: return "Unknown error";
    }
}

void _us_mpp_update_stats(us_mpp_processor_s *proc, uint64_t process_time_us, bool success, bool is_encode) {
    if (!proc) return;
    
    uint64_t now = _us_mpp_get_time_us();
    proc->stats.frames_processed++;
    proc->stats.total_processing_time_ms += process_time_us / 1000.0;
    proc->stats.avg_processing_time_ms = proc->stats.total_processing_time_ms / proc->stats.frames_processed;
    
    if (success) {
        if (is_encode) {
            proc->stats.frames_encoded++;
        } else {
            proc->stats.frames_decoded++;
        }
        proc->consecutive_errors = 0;
    } else {
        proc->stats.processing_errors++;
        if (is_encode) {
            proc->stats.encode_errors++;
        } else {
            proc->stats.decode_errors++;
        }
        proc->consecutive_errors++;
    }
    
    // 更新FPS (每秒计算一次)
    if (now - proc->stats.last_stats_update >= 1000000) {
        double elapsed_sec = (now - proc->stats.last_stats_update) / 1000000.0;
        if (elapsed_sec > 0) {
            // 简化FPS计算，使用处理时间倒数估算
            proc->stats.current_fps = (proc->stats.avg_processing_time_ms > 0) ? 
                (1000.0 / proc->stats.avg_processing_time_ms) : 0;
        }
        proc->stats.last_stats_update = now;
    }
}

us_mpp_error_e _us_mpp_init_buffer_manager(us_mpp_buffer_mgr_s *mgr, uint32_t buffer_count, uint32_t buffer_size) {
    US_MPP_CHECK_NULL(mgr, "Buffer manager");
    
    memset(mgr, 0, sizeof(us_mpp_buffer_mgr_s));
    
    if (pthread_mutex_init(&mgr->mutex, NULL) != 0) {
        US_MPP_LOG_ERROR("Failed to initialize buffer manager mutex");
        return US_MPP_ERROR_MEMORY;
    }
    
    // 创建MPP缓冲区组 - 按照MPP测试使用DRM+CACHABLE（关键优化）
    MPP_RET ret = mpp_buffer_group_get_external(&mgr->buffer_group, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
    if (ret != MPP_OK) {
        US_MPP_LOG_ERROR("Failed to get MPP buffer group: %d", ret);
        pthread_mutex_destroy(&mgr->mutex);
        return US_MPP_ERROR_MEMORY;
    }
    
    mgr->input_buffer_count = buffer_count;
    mgr->output_buffer_count = buffer_count;
    mgr->buffer_size = buffer_size;
    
    // 预分配输入缓冲区
    mgr->input_buffers = calloc(buffer_count, sizeof(MppBuffer));
    mgr->output_buffers = calloc(buffer_count, sizeof(MppBuffer));
    if (!mgr->input_buffers || !mgr->output_buffers) {
        US_MPP_LOG_ERROR("Failed to allocate buffer arrays");
        free(mgr->input_buffers);
        free(mgr->output_buffers);
        mpp_buffer_group_put(mgr->buffer_group);
        pthread_mutex_destroy(&mgr->mutex);
        return US_MPP_ERROR_MEMORY;
    }
    
    for (uint32_t i = 0; i < buffer_count; i++) {
        ret = mpp_buffer_get(mgr->buffer_group, &mgr->input_buffers[i], buffer_size);
        if (ret != MPP_OK) {
            US_MPP_LOG_ERROR("Failed to allocate input buffer %u: %d", i, ret);
            // 清理已分配的缓冲区
            for (uint32_t j = 0; j < i; j++) {
                if (mgr->input_buffers[j]) {
                    mpp_buffer_put(mgr->input_buffers[j]);
                }
            }
            free(mgr->input_buffers);
            free(mgr->output_buffers);
            mpp_buffer_group_put(mgr->buffer_group);
            pthread_mutex_destroy(&mgr->mutex);
            return US_MPP_ERROR_MEMORY;
        }
        
        ret = mpp_buffer_get(mgr->buffer_group, &mgr->output_buffers[i], buffer_size);
        if (ret != MPP_OK) {
            US_MPP_LOG_ERROR("Failed to allocate output buffer %u: %d", i, ret);
            // 清理已分配的缓冲区
            for (uint32_t j = 0; j <= i; j++) {
                if (mgr->input_buffers[j]) {
                    mpp_buffer_put(mgr->input_buffers[j]);
                }
                if (j < i && mgr->output_buffers[j]) {
                    mpp_buffer_put(mgr->output_buffers[j]);
                }
            }
            free(mgr->input_buffers);
            free(mgr->output_buffers);
            mpp_buffer_group_put(mgr->buffer_group);
            pthread_mutex_destroy(&mgr->mutex);
            return US_MPP_ERROR_MEMORY;
        }
    }
    
    US_MPP_LOG_INFO("Buffer manager initialized: %u buffers, %u bytes each", buffer_count, buffer_size);
    return US_MPP_OK;
}

static void _us_mpp_deinit_buffer_manager(us_mpp_buffer_mgr_s *mgr) {
    if (!mgr) return;
    
    pthread_mutex_lock(&mgr->mutex);
    
    // 释放所有缓冲区
    if (mgr->input_buffers) {
        for (uint32_t i = 0; i < mgr->input_buffer_count; i++) {
            if (mgr->input_buffers[i]) {
                mpp_buffer_put(mgr->input_buffers[i]);
            }
        }
        free(mgr->input_buffers);
        mgr->input_buffers = NULL;
    }
    
    if (mgr->output_buffers) {
        for (uint32_t i = 0; i < mgr->output_buffer_count; i++) {
            if (mgr->output_buffers[i]) {
                mpp_buffer_put(mgr->output_buffers[i]);
            }
        }
        free(mgr->output_buffers);
        mgr->output_buffers = NULL;
    }
    
    if (mgr->buffer_group) {
        mpp_buffer_group_put(mgr->buffer_group);
        mgr->buffer_group = NULL;
    }
    
    pthread_mutex_unlock(&mgr->mutex);
    pthread_mutex_destroy(&mgr->mutex);
    
    memset(mgr, 0, sizeof(us_mpp_buffer_mgr_s));
}

// ==== 核心处理器基础函数 ====

us_mpp_error_e _us_mpp_processor_init_base(us_mpp_processor_s *proc, us_mpp_codec_type_e type) {
    US_MPP_CHECK_NULL(proc, "Processor");
    
    memset(proc, 0, sizeof(us_mpp_processor_s));
    
    proc->codec_type = type;
    proc->max_consecutive_errors = US_MPP_MAX_CONSECUTIVE_ERRORS;
    proc->debug_level = 1;
    snprintf(proc->debug_prefix, sizeof(proc->debug_prefix), "[MPP-%s]", 
             type == US_MPP_CODEC_MJPEG_DEC ? "DEC" : "ENC");
    
    // 初始化互斥锁
    if (pthread_mutex_init(&proc->mutex, NULL) != 0) {
        US_MPP_LOG_ERROR("Failed to initialize processor mutex");
        return US_MPP_ERROR_MEMORY;
    }
    proc->mutex_initialized = true;
    
    // 创建MPP上下文
    MPP_RET ret = mpp_create(&proc->ctx, &proc->mpi);
    if (ret != MPP_OK) {
        US_MPP_LOG_ERROR("Failed to create MPP context: %d", ret);
        pthread_mutex_destroy(&proc->mutex);
        return US_MPP_ERROR_INIT;
    }
    
    // 创建packet和frame
    ret = mpp_packet_init(&proc->packet, NULL, 0);
    if (ret != MPP_OK) {
        US_MPP_LOG_ERROR("Failed to init MPP packet: %d", ret);
        mpp_destroy(proc->ctx);
        pthread_mutex_destroy(&proc->mutex);
        return US_MPP_ERROR_INIT;
    }
    
    ret = mpp_frame_init(&proc->frame);
    if (ret != MPP_OK) {
        US_MPP_LOG_ERROR("Failed to init MPP frame: %d", ret);
        mpp_packet_deinit(&proc->packet);
        mpp_destroy(proc->ctx);
        pthread_mutex_destroy(&proc->mutex);
        return US_MPP_ERROR_INIT;
    }
    
    proc->stats.last_stats_update = _us_mpp_get_time_us();
    
    US_MPP_LOG_INFO("MPP processor base initialized for %s", 
                   type == US_MPP_CODEC_MJPEG_DEC ? "MJPEG decoder" : "H264 encoder");
    
    return US_MPP_OK;
}

// ==== 公共API实现 ====

const char* us_mpp_error_string(us_mpp_error_e error) {
    return _us_mpp_error_to_string(error);
}

const char* us_mpp_codec_type_string(us_mpp_codec_type_e type) {
    switch (type) {
        case US_MPP_CODEC_MJPEG_DEC: return "MJPEG Decoder";
        case US_MPP_CODEC_H264_ENC: return "H264 Encoder";
        case US_MPP_CODEC_H265_ENC: return "H265 Encoder";
        default: return "Unknown";
    }
}

bool us_mpp_is_format_supported_for_decode(uint32_t format) {
    // 支持MJPEG解码
    return (format == V4L2_PIX_FMT_MJPEG || format == V4L2_PIX_FMT_JPEG);
}

bool us_mpp_is_format_supported_for_encode(uint32_t format) {
    // 支持NV12输入进行H264编码
    return (format == V4L2_PIX_FMT_NV12);
}

void us_mpp_processor_destroy(us_mpp_processor_s *processor) {
    if (!processor) return;
    
    US_MPP_LOG_INFO("Destroying MPP processor (%s)", 
                   us_mpp_codec_type_string(processor->codec_type));
    
    // 设置停止标志
    processor->should_stop = true;
    
    if (processor->mutex_initialized) {
        pthread_mutex_lock(&processor->mutex);
    }
    
    // 先尝试复位MPP上下文，避免驱动持有资源（与MPP示例一致）
    if (processor->ctx && processor->mpi) {
        processor->mpi->reset(processor->ctx);
    }

    // 清理缓冲区管理器
    _us_mpp_deinit_buffer_manager(&processor->buffer_mgr);
    
    // 清理编码配置
    if (processor->enc_cfg) {
        mpp_enc_cfg_deinit(processor->enc_cfg);
        processor->enc_cfg = NULL;
    }
    
    // 清理MPP资源
    if (processor->packet) {
        mpp_packet_deinit(&processor->packet);
    }
    if (processor->frame) {
        mpp_frame_deinit(&processor->frame);
    }
    
    // 清理MJPEG解码器专用的buffer group
    if (processor->frm_buf) {
        mpp_buffer_put(processor->frm_buf);
        processor->frm_buf = NULL;
    }
    if (processor->frm_grp) {
        mpp_buffer_group_put(processor->frm_grp);
        processor->frm_grp = NULL;
    }
    if (processor->pkt_grp) {
        mpp_buffer_group_put(processor->pkt_grp);
        processor->pkt_grp = NULL;
    }
    
    if (processor->ctx) {
        mpp_destroy(processor->ctx);
        processor->ctx = NULL;
        processor->mpi = NULL;
    }
    
    atomic_store(&processor->initialized, false);
    
    if (processor->mutex_initialized) {
        pthread_mutex_unlock(&processor->mutex);
        pthread_mutex_destroy(&processor->mutex);
    }
    
    // 打印最终统计信息
    US_MPP_LOG_INFO("Final stats - Processed: %lu, Errors: %lu, Avg time: %.2f ms", 
                   processor->stats.frames_processed,
                   processor->stats.processing_errors,
                   processor->stats.avg_processing_time_ms);
    
    free(processor);
}

us_mpp_error_e us_mpp_processor_get_stats(us_mpp_processor_s *processor, us_mpp_stats_s *stats) {
    US_MPP_CHECK_NULL(processor, "Processor");
    US_MPP_CHECK_NULL(stats, "Stats");
    US_MPP_CHECK_INIT(processor, "Processor");
    
    pthread_mutex_lock(&processor->mutex);
    *stats = processor->stats;
    pthread_mutex_unlock(&processor->mutex);
    
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_processor_reset(us_mpp_processor_s *processor) {
    US_MPP_CHECK_NULL(processor, "Processor");
    US_MPP_CHECK_INIT(processor, "Processor");
    
    pthread_mutex_lock(&processor->mutex);
    
    // 重置MPP上下文
    MPP_RET ret = processor->mpi->reset(processor->ctx);
    if (ret != MPP_OK) {
        US_MPP_LOG_ERROR("Failed to reset MPP context: %d", ret);
        pthread_mutex_unlock(&processor->mutex);
        return US_MPP_ERROR_INIT;
    }
    
    // 重置统计信息
    memset(&processor->stats, 0, sizeof(us_mpp_stats_s));
    processor->stats.last_stats_update = _us_mpp_get_time_us();
    processor->consecutive_errors = 0;
    processor->frame_number = 0;
    
    pthread_mutex_unlock(&processor->mutex);
    
    US_MPP_LOG_INFO("MPP processor reset successfully");
    return US_MPP_OK;
}

// ==== 配置函数 ====

us_mpp_error_e us_mpp_processor_set_debug_level(us_mpp_processor_s *processor, uint32_t level) {
    US_MPP_CHECK_NULL(processor, "Processor");
    
    processor->debug_level = level;
    US_MPP_LOG_INFO("Debug level set to %u", level);
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_processor_enable_zero_copy(us_mpp_processor_s *processor, bool enable) {
    US_MPP_CHECK_NULL(processor, "Processor");
    
    processor->zero_copy_enabled = enable;
    US_MPP_LOG_INFO("Zero-copy %s", enable ? "enabled" : "disabled");
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_processor_enable_parallel(us_mpp_processor_s *processor, bool enable) {
    US_MPP_CHECK_NULL(processor, "Processor");
    
    processor->parallel_processing = enable;
    US_MPP_LOG_INFO("Parallel processing %s", enable ? "enabled" : "disabled");
    return US_MPP_OK;
}
