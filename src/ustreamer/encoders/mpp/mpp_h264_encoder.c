#include "mpp_encoder.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <linux/videodev2.h>

// MPP头文件
#include "rk_mpi.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "mpp_meta.h"
#include "rk_venc_cfg.h"

// uStreamer内部头文件
#include "../../../libs/logging.h"
#include "../../../libs/tools.h"
#include "../../../libs/frame.h"

// 前向声明（这些函数在mpp_encoder.c中实现）
us_mpp_error_e _us_mpp_processor_init_base(us_mpp_processor_s *proc, us_mpp_codec_type_e type);
us_mpp_error_e _us_mpp_init_buffer_manager(us_mpp_buffer_mgr_s *mgr, uint32_t buffer_count, uint32_t buffer_size);
void _us_mpp_update_stats(us_mpp_processor_s *proc, uint64_t process_time_us, bool success, bool is_encode);
uint64_t _us_mpp_get_time_us(void);

#define US_MPP_H264_LOG_PREFIX "[MPP-H264-ENC] "
#define US_MPP_H264_LOG_INFO(fmt, ...) US_LOG_INFO(US_MPP_H264_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_H264_LOG_ERROR(fmt, ...) US_LOG_ERROR(US_MPP_H264_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_H264_LOG_DEBUG(fmt, ...) US_LOG_DEBUG(US_MPP_H264_LOG_PREFIX fmt, ##__VA_ARGS__)


#define US_MPP_H264_MAX_RETRY 30

// H264编码器内部函数
static us_mpp_error_e _us_mpp_h264_setup_encoder(us_mpp_processor_s *encoder) {
    if (!encoder || !encoder->mpi || !encoder->ctx) {
        US_MPP_H264_LOG_ERROR("Invalid encoder context");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    // 初始化H264编码器 - 使用mpp_init而不是control命令
    MppCodingType coding = MPP_VIDEO_CodingAVC;
    MPP_RET ret = mpp_init(encoder->ctx, MPP_CTX_ENC, coding);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to init H264 encoder: %d", ret);
        return US_MPP_ERROR_INIT;
    }

    // 设置超时以防止永久阻塞（关键修复）
    MppPollType timeout = US_MPP_TIMEOUT_MS;
    ret = encoder->mpi->control(encoder->ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to set output timeout: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    // 初始化编码器配置
    ret = mpp_enc_cfg_init(&encoder->enc_cfg);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to init encoder config: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    // 获取默认配置
    ret = encoder->mpi->control(encoder->ctx, MPP_ENC_GET_CFG, encoder->enc_cfg);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to get encoder config: %d", ret);
        mpp_enc_cfg_deinit(encoder->enc_cfg);
        encoder->enc_cfg = NULL;
        return US_MPP_ERROR_INIT;
    }
    
    US_MPP_H264_LOG_INFO("H264 encoder setup completed");
    return US_MPP_OK;
}

static us_mpp_error_e _us_mpp_h264_configure_encoder(us_mpp_processor_s *encoder) {
    if (!encoder || !encoder->enc_cfg) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    MPP_RET ret = MPP_OK;
    
    // 基本参数配置
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "prep:width", encoder->width);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "prep:height", encoder->height);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "prep:hor_stride", encoder->hor_stride);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "prep:ver_stride", encoder->ver_stride);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "prep:format", MPP_FMT_YUV420SP);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "prep:range", MPP_FRAME_RANGE_JPEG);  // 按照MPP测试添加
    
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to set prep config: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    // 码率控制配置
    ret |= mpp_enc_cfg_set_u32(encoder->enc_cfg, "rc:mode", encoder->rc_mode);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:bps_target", encoder->bitrate_bps);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:bps_max", encoder->bitrate_bps * 1.2);  // 允许20%超调
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:bps_min", encoder->bitrate_bps * 0.8);  // 允许20%低调
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:fps_in_flex", 0);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:fps_in_num", encoder->fps_num);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:fps_in_denom", encoder->fps_den);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:fps_out_flex", 0);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:fps_out_num", encoder->fps_num);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:fps_out_denom", encoder->fps_den);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "rc:gop", encoder->gop_size);
    
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to set rc config: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    // H264编码器特定配置
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:profile", encoder->profile);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:level", encoder->level);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:cabac_en", 1);          // 启用CABAC
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:cabac_idc", 0);         // CABAC IDC
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:trans8x8", 1);          // 启用8x8变换
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:qp_init", encoder->qp_init);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:qp_max", encoder->qp_max);
    ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:qp_min", encoder->qp_min);
    
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to set h264 config: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    // 应用配置
    ret = encoder->mpi->control(encoder->ctx, MPP_ENC_SET_CFG, encoder->enc_cfg);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to apply encoder config: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    US_MPP_H264_LOG_INFO("Encoder configured: %ux%u, %u kbps, GOP %u, Profile %u, QP %u-%u",
                        encoder->width, encoder->height, 
                        encoder->bitrate_bps / 1000, encoder->gop_size,
                        encoder->profile, encoder->qp_min, encoder->qp_max);
    
    return US_MPP_OK;
}

static us_mpp_error_e _us_mpp_h264_setup_input_frame(us_mpp_processor_s *encoder, const us_frame_s *nv12_frame, bool force_key) {
    if (!encoder || !nv12_frame) {
        return US_MPP_ERROR_INVALID_PARAM;
    }

    mpp_frame_set_width(encoder->frame, nv12_frame->width);
    mpp_frame_set_height(encoder->frame, nv12_frame->height);
    mpp_frame_set_hor_stride(encoder->frame, encoder->hor_stride);
    mpp_frame_set_ver_stride(encoder->frame, encoder->ver_stride);
    mpp_frame_set_fmt(encoder->frame, MPP_FMT_YUV420SP);
    mpp_frame_set_eos(encoder->frame, 0);

    // 关键修改：直接使用预分配的frm_buf
    mpp_frame_set_buffer(encoder->frame, encoder->frm_buf);

    // 关键修复：参考mpi_enc_test.c line 850-854，正确创建输出packet
    MppMeta meta = mpp_frame_get_meta(encoder->frame);
    
    // 为输出创建packet（使用预分配的pkt_buf）
    mpp_packet_init_with_buffer(&encoder->packet, encoder->pkt_buf);
    // 重要：清空输出packet长度！
    mpp_packet_set_length(encoder->packet, 0);
    mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, encoder->packet);
    mpp_meta_set_buffer(meta, KEY_MOTION_INFO, NULL);

    if (force_key) {
        // 使用KEY_OUTPUT_INTRA来强制I帧
        mpp_meta_set_s32(meta, KEY_OUTPUT_INTRA, 1);
        US_MPP_H264_LOG_DEBUG("Forcing keyframe");
    }

    return US_MPP_OK;
}

static us_mpp_error_e _us_mpp_h264_extract_output_packet(us_mpp_processor_s *encoder, MppPacket packet, us_frame_s *h264_frame) {
    if (!encoder || !packet || !h264_frame) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    void *data = mpp_packet_get_data(packet);
    size_t length = mpp_packet_get_length(packet);
    
    if (!data || length == 0) {
        US_MPP_H264_LOG_ERROR("Empty output packet");
        return US_MPP_ERROR_ENCODE;
    }
    
    // 设置输出帧参数
    h264_frame->format = V4L2_PIX_FMT_H264;
    h264_frame->width = encoder->width;
    h264_frame->height = encoder->height;
    h264_frame->stride = 0;  // H264流没有stride概念
    h264_frame->used = length;
    
    // 分配输出缓冲区
    us_frame_realloc_data(h264_frame, length);
    if (!h264_frame->data) {
        US_MPP_H264_LOG_ERROR("Failed to allocate output buffer");
        return US_MPP_ERROR_MEMORY;
    }
    
    // 拷贝H264数据
    memcpy(h264_frame->data, data, length);
    
    // 检查是否为关键帧 - 从meta数据获取
    bool is_keyframe = false;
    MppMeta meta = mpp_packet_get_meta(packet);
    if (meta) {
        RK_S32 is_intra = 0;
        mpp_meta_get_s32(meta, KEY_OUTPUT_INTRA, &is_intra);
        if (is_intra) {
            is_keyframe = true;
            encoder->stats.keyframes_generated++;
        }
    }
    
    US_MPP_H264_LOG_DEBUG("H264 packet extracted: %zu bytes, %s", length, 
                         is_keyframe ? "KEYFRAME" : "P-FRAME");
    
    return US_MPP_OK;
}

// ==== H264编码器公共API ====

us_mpp_error_e us_mpp_h264_encoder_create(us_mpp_processor_s **encoder,
                                         uint32_t width, uint32_t height,
                                         uint32_t bitrate_kbps, uint32_t gop_size,
                                         uint32_t fps_num, uint32_t fps_den) {
    if (!encoder) {
        US_MPP_H264_LOG_ERROR("Encoder pointer is NULL");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (width == 0 || height == 0 || bitrate_kbps == 0 || fps_num == 0 || fps_den == 0) {
        US_MPP_H264_LOG_ERROR("Invalid parameters: %ux%u, %u kbps, %u/%u fps", 
                             width, height, bitrate_kbps, fps_num, fps_den);
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    *encoder = NULL;
    
    // 分配编码器结构
    us_mpp_processor_s *enc = calloc(1, sizeof(us_mpp_processor_s));
    if (!enc) {
        US_MPP_H264_LOG_ERROR("Failed to allocate encoder structure");
        return US_MPP_ERROR_MEMORY;
    }
    
    // 初始化基础处理器
    us_mpp_error_e err = _us_mpp_processor_init_base(enc, US_MPP_CODEC_H264_ENC);
    if (err != US_MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to initialize base processor: %s", us_mpp_error_string(err));
        free(enc);
        return err;
    }
    
    // 设置编码器参数
    enc->width = width;
    enc->height = height;
    enc->hor_stride = (width + 15) & (~15);     // 16字节对齐
    enc->ver_stride = (height + 15) & (~15);    // 16字节对齐
    enc->bitrate_bps = bitrate_kbps * 1000;
    enc->fps_num = fps_num;
    enc->fps_den = fps_den;
    enc->gop_size = gop_size;
    
    // H264默认参数
    enc->profile = 100;         // High Profile
    enc->level = 40;            // Level 4.0
    enc->rc_mode = 1;           // CBR模式
    enc->qp_init = 24;          // 初始QP
    enc->qp_min = 16;           // 最小QP
    enc->qp_max = 40;           // 最大QP
    
    enc->zero_copy_enabled = true;  // 默认启用零拷贝
    
    // 初始化H264编码器
    err = _us_mpp_h264_setup_encoder(enc);
    if (err != US_MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to setup H264 encoder: %s", us_mpp_error_string(err));
        us_mpp_processor_destroy(enc);
        return err;
    }
    
    // 配置编码器参数
    err = _us_mpp_h264_configure_encoder(enc);
    if (err != US_MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to configure H264 encoder: %s", us_mpp_error_string(err));
        us_mpp_processor_destroy(enc);
        return err;
    }
    
    // 移除buffer_manager，改为与MPP官方示例一致的单个输入buffer模式
    uint32_t frame_size = _us_mpp_calc_frame_size(enc->width, enc->height, MPP_FMT_YUV420SP);
    
    // 关键修复：完全对齐MPP官方测试用例，使用internal group
    MPP_RET ret = mpp_buffer_group_get_internal(&enc->frm_grp, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to create internal buffer group for encoder: %d", ret);
        us_mpp_processor_destroy(enc);
        return US_MPP_ERROR_MEMORY;
    }
    
    ret = mpp_buffer_get(enc->frm_grp, &enc->frm_buf, frame_size);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to allocate frame buffer for encoder: %d", ret);
        us_mpp_processor_destroy(enc);
        return US_MPP_ERROR_MEMORY;
    }
    
    // 关键修复：为输出packet分配buffer（参考mpi_enc_test.c line 1133）
    ret = mpp_buffer_get(enc->frm_grp, &enc->pkt_buf, frame_size);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to allocate packet buffer for encoder: %d", ret);
        us_mpp_processor_destroy(enc);
        return US_MPP_ERROR_MEMORY;
    }
    
    US_MPP_H264_LOG_INFO("Allocated buffers: frame=%zu bytes, packet=%zu bytes", frame_size, frame_size);
    
    atomic_store(&enc->initialized, true);
    *encoder = enc;
    
    US_MPP_H264_LOG_INFO("H264 encoder created successfully: %ux%u, %u kbps, GOP %u, %u/%u fps",
                        width, height, bitrate_kbps, gop_size, fps_num, fps_den);
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_h264_encoder_encode(us_mpp_processor_s *encoder,
                                         const us_frame_s *nv12_frame,
                                         us_frame_s *h264_frame,
                                         bool force_key) {
    if (!encoder || !nv12_frame || !h264_frame) {
        US_MPP_H264_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!atomic_load(&encoder->initialized)) {
        US_MPP_H264_LOG_ERROR("Encoder not initialized");
        return US_MPP_ERROR_NOT_INITIALIZED;
    }
    
    if (!us_mpp_is_format_supported_for_encode(nv12_frame->format)) {
        US_MPP_H264_LOG_ERROR("Unsupported input format: %u", nv12_frame->format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    if (nv12_frame->used == 0 || !nv12_frame->data) {
        US_MPP_H264_LOG_ERROR("Empty NV12 frame data");
        return US_MPP_ERROR_INVALID_PARAM;
    }

    uint64_t start_time = _us_mpp_get_time_us();
    us_mpp_error_e result = US_MPP_OK;
    MPP_RET ret = MPP_OK;

    pthread_mutex_lock(&encoder->mutex);

    if (encoder->should_stop) {
        pthread_mutex_unlock(&encoder->mutex);
        return US_MPP_ERROR_NOT_INITIALIZED;
    }

    atomic_store(&encoder->processing, true);

    // 将输入数据拷贝到MPP buffer中
    void *buf = mpp_buffer_get_ptr(encoder->frm_buf);
    memcpy(buf, nv12_frame->data, nv12_frame->used);
    mpp_buffer_sync_end(encoder->frm_buf);

    // 设置输入帧
    us_mpp_error_e err = _us_mpp_h264_setup_input_frame(encoder, nv12_frame, force_key);
    if (err != US_MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to setup input frame: %s", us_mpp_error_string(err));
        result = err;
        goto cleanup;
    }

    // 发送帧进行编码
    ret = encoder->mpi->encode_put_frame(encoder->ctx, encoder->frame);
    if (ret != MPP_OK) {
        US_MPP_H264_LOG_ERROR("Failed to put encode frame: %d", ret);
        result = US_MPP_ERROR_ENCODE;
        goto cleanup;
    }
    
    // 获取编码结果
    MppPacket packet = NULL;
    int retry_count = 0;
    
    do {
        // 获取编码后的数据包
        ret = encoder->mpi->encode_get_packet(encoder->ctx, &packet);
        if (ret == MPP_ERR_TIMEOUT) {
            US_MPP_H264_LOG_DEBUG("Get packet timeout, assuming all packets received");
            break; // 超时意味着当前帧没有更多数据包，正常退出
        }
        if (ret != MPP_OK) {
            US_MPP_H264_LOG_ERROR("Failed to get encode packet: %d", ret);
            result = US_MPP_ERROR_ENCODE;
            break;
        }
        
        if (packet) {
            // 提取H264数据
            us_mpp_error_e extract_err = _us_mpp_h264_extract_output_packet(encoder, packet, h264_frame);
            // 关键修复：正确销毁packet（参考mpi_enc_test.c line 1070）
            mpp_packet_deinit(&packet);
            
            if (extract_err != US_MPP_OK) {
                US_MPP_H264_LOG_ERROR("Failed to extract output packet: %s", us_mpp_error_string(extract_err));
                result = extract_err;
            }
            break;
        }
        
        retry_count++;
        if (retry_count >= US_MPP_H264_MAX_RETRY) {
            US_MPP_H264_LOG_ERROR("No packet received after %d retries", retry_count);
            result = US_MPP_ERROR_TIMEOUT;
            break;
        }
        
        usleep(1000);  // 等待1ms
        
    } while (1);
    
cleanup:
    atomic_store(&encoder->processing, false);
    encoder->frame_number++;
    
    uint64_t end_time = _us_mpp_get_time_us();
    uint64_t process_time = end_time - start_time;
    
    _us_mpp_update_stats(encoder, process_time, (result == US_MPP_OK), true);
    
    pthread_mutex_unlock(&encoder->mutex);
    
    if (result == US_MPP_OK) {
        US_MPP_H264_LOG_DEBUG("H264 encode success: %ux%u NV12 -> %zu bytes H264 (%.2f ms) %s", 
                             nv12_frame->width, nv12_frame->height,
                             h264_frame->used, process_time / 1000.0,
                             force_key ? "[FORCED KEY]" : "");
    } else {
        US_MPP_H264_LOG_ERROR("H264 encode failed: %s (%.2f ms)", 
                             us_mpp_error_string(result), process_time / 1000.0);
    }
    
    return result;
}

// ==== H264编码器高级配置API ====

us_mpp_error_e us_mpp_h264_encoder_set_profile(us_mpp_processor_s *encoder, uint32_t profile) {
    if (!encoder) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!atomic_load(&encoder->initialized)) {
        encoder->profile = profile;
        return US_MPP_OK;
    }
    
    // 运行时更新配置
    pthread_mutex_lock(&encoder->mutex);
    encoder->profile = profile;
    
    if (encoder->enc_cfg) {
        MPP_RET ret = mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:profile", profile);
        if (ret == MPP_OK) {
            ret = encoder->mpi->control(encoder->ctx, MPP_ENC_SET_CFG, encoder->enc_cfg);
        }
        if (ret != MPP_OK) {
            US_MPP_H264_LOG_ERROR("Failed to update profile: %d", ret);
            pthread_mutex_unlock(&encoder->mutex);
            return US_MPP_ERROR_INIT;
        }
    }
    
    pthread_mutex_unlock(&encoder->mutex);
    US_MPP_H264_LOG_INFO("Profile updated to %u", profile);
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_h264_encoder_set_rc_mode(us_mpp_processor_s *encoder, uint32_t rc_mode) {
    if (!encoder) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!atomic_load(&encoder->initialized)) {
        encoder->rc_mode = rc_mode;
        return US_MPP_OK;
    }
    
    // 运行时更新配置
    pthread_mutex_lock(&encoder->mutex);
    encoder->rc_mode = rc_mode;
    
    if (encoder->enc_cfg) {
        MPP_RET ret = mpp_enc_cfg_set_u32(encoder->enc_cfg, "rc:mode", rc_mode);
        if (ret == MPP_OK) {
            ret = encoder->mpi->control(encoder->ctx, MPP_ENC_SET_CFG, encoder->enc_cfg);
        }
        if (ret != MPP_OK) {
            US_MPP_H264_LOG_ERROR("Failed to update rc mode: %d", ret);
            pthread_mutex_unlock(&encoder->mutex);
            return US_MPP_ERROR_INIT;
        }
    }
    
    pthread_mutex_unlock(&encoder->mutex);
    US_MPP_H264_LOG_INFO("RC mode updated to %u", rc_mode);
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_h264_encoder_set_qp_range(us_mpp_processor_s *encoder, uint32_t qp_min, uint32_t qp_max) {
    if (!encoder || qp_min > qp_max || qp_max > 51) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!atomic_load(&encoder->initialized)) {
        encoder->qp_min = qp_min;
        encoder->qp_max = qp_max;
        return US_MPP_OK;
    }
    
    // 运行时更新配置
    pthread_mutex_lock(&encoder->mutex);
    encoder->qp_min = qp_min;
    encoder->qp_max = qp_max;
    
    if (encoder->enc_cfg) {
        MPP_RET ret = mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:qp_min", qp_min);
        ret |= mpp_enc_cfg_set_s32(encoder->enc_cfg, "h264:qp_max", qp_max);
        if (ret == MPP_OK) {
            ret = encoder->mpi->control(encoder->ctx, MPP_ENC_SET_CFG, encoder->enc_cfg);
        }
        if (ret != MPP_OK) {
            US_MPP_H264_LOG_ERROR("Failed to update QP range: %d", ret);
            pthread_mutex_unlock(&encoder->mutex);
            return US_MPP_ERROR_INIT;
        }
    }
    
    pthread_mutex_unlock(&encoder->mutex);
    US_MPP_H264_LOG_INFO("QP range updated to %u-%u", qp_min, qp_max);
    return US_MPP_OK;
}
