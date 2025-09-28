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

// uStreamer内部头文件
#include "../../../libs/logging.h"
#include "../../../libs/tools.h"
#include "../../../libs/frame.h"

#define US_MPP_MJPEG_LOG_PREFIX "[MPP-MJPEG-DEC] "
#define US_MPP_MJPEG_LOG_INFO(fmt, ...) US_LOG_INFO(US_MPP_MJPEG_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_MJPEG_LOG_ERROR(fmt, ...) US_LOG_ERROR(US_MPP_MJPEG_LOG_PREFIX fmt, ##__VA_ARGS__)
#define US_MPP_MJPEG_LOG_DEBUG(fmt, ...) US_LOG_DEBUG(US_MPP_MJPEG_LOG_PREFIX fmt, ##__VA_ARGS__)


#define US_MPP_MJPEG_TIMEOUT_MS 100
#define US_MPP_MJPEG_MAX_RETRY 30

// MJPEG解码器内部函数
static us_mpp_error_e _us_mpp_mjpeg_setup_decoder(us_mpp_processor_s *decoder) {
    if (!decoder || !decoder->mpi || !decoder->ctx) {
        US_MPP_MJPEG_LOG_ERROR("Invalid decoder context");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    // 初始化MJPEG解码器 - 使用mpp_init而不是control命令
    MppCodingType coding = MPP_VIDEO_CodingMJPEG;
    MPP_RET ret = mpp_init(decoder->ctx, MPP_CTX_DEC, coding);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to init MJPEG decoder: %d", ret);
        return US_MPP_ERROR_INIT;
    }

    // 设置超时以防止永久阻塞（关键修复）
    MppPollType timeout = US_MPP_TIMEOUT_MS;
    ret = decoder->mpi->control(decoder->ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to set output timeout: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    // 关键修复：MJPEG需要预设输出格式（参考mpi_dec_test.c line 382-388）
    MppFrameFormat fmt = MPP_FMT_YUV420SP;  // NV12 = YUV420SP
    ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_OUTPUT_FORMAT, &fmt);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to set output format to NV12: %d", ret);
        return US_MPP_ERROR_INIT;
    }
    
    // 同步MPP解码示例：开启split_parse，保证内部按帧切分（关键优化）
    MppDecCfg dec_cfg = NULL;
    ret = mpp_dec_cfg_init(&dec_cfg);
    if (ret == MPP_OK) {
        ret = decoder->mpi->control(decoder->ctx, MPP_DEC_GET_CFG, dec_cfg);
        if (ret == MPP_OK) {
            (void)mpp_dec_cfg_set_u32(dec_cfg, "base:split_parse", 1);
            ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_CFG, dec_cfg);
            if (ret != MPP_OK) {
                US_MPP_MJPEG_LOG_ERROR("Failed to set decoder cfg: %d", ret);
            }
        } else {
            US_MPP_MJPEG_LOG_ERROR("Failed to get decoder cfg: %d", ret);
        }
        mpp_dec_cfg_deinit(dec_cfg);
    } else {
        US_MPP_MJPEG_LOG_ERROR("Failed to init decoder cfg: %d", ret);
    }
    
    // MJPEG解码器使用简化setup - 让MPP在info_change时自动分配buffer（关键修复）
    // 避免预分配buffer group以防止CMA内存分配失败
    
    US_MPP_MJPEG_LOG_INFO("MJPEG decoder setup completed");
    return US_MPP_OK;
}

static us_mpp_error_e _us_mpp_mjpeg_process_info_change(us_mpp_processor_s *decoder, MppFrame frame) {
    if (!decoder || !frame) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (mpp_frame_get_info_change(frame)) {
        decoder->width = mpp_frame_get_width(frame);
        decoder->height = mpp_frame_get_height(frame);
        decoder->hor_stride = mpp_frame_get_hor_stride(frame);
        decoder->ver_stride = mpp_frame_get_ver_stride(frame);
        uint32_t buf_size = mpp_frame_get_buf_size(frame);
        
        US_MPP_MJPEG_LOG_INFO("Info change: %ux%u, stride: %ux%u, buf_size: %u",
                             decoder->width, decoder->height,
                             decoder->hor_stride, decoder->ver_stride, buf_size);
        
        // 按照MPP测试的正确方式：创建external buffer group（关键修复）
        if (!decoder->frm_grp) {
            MPP_RET ret = mpp_buffer_group_get_external(&decoder->frm_grp, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
            if (ret != MPP_OK) {
                US_MPP_MJPEG_LOG_ERROR("Failed to get external buffer group: %d", ret);
                return US_MPP_ERROR_INIT;
            }
            
            // 关键修复：MJPEG需要更大的buffer（参考mpi_dec_test.c line 484-495）
            // JPEG可能有YUV420和YUV422，buffer应该是4倍大小，并且16字节对齐
            uint32_t hor_stride = MPP_ALIGN(decoder->width, 16);
            uint32_t ver_stride = MPP_ALIGN(decoder->height, 16); 
            uint32_t jpeg_buf_size = hor_stride * ver_stride * 4;  // 4倍大小支持YUV422
            
            US_MPP_MJPEG_LOG_INFO("MJPEG buffer: %ux%u -> %ux%u (stride), size: %u bytes", 
                                 decoder->width, decoder->height, hor_stride, ver_stride, jpeg_buf_size);
            
            // 预分配24个buffer（参考MPP测试mpi_dec_test.c line 155）
            int successful_buffers = 0;
            for (int i = 0; i < 24; i++) {
                MppBuffer buffer = NULL;
                ret = mpp_buffer_get(decoder->frm_grp, &buffer, jpeg_buf_size);
                if (ret != MPP_OK) {
                    US_MPP_MJPEG_LOG_DEBUG("Failed to get buffer %d: %d (got %d buffers)", i, ret, successful_buffers);
                    break; // 部分成功也可以工作，但记录实际分配数量
                }
                mpp_buffer_put(buffer);  // 立即释放引用，让group管理
                successful_buffers++;
            }
            
            // 设置buffer group到decoder（关键步骤）
            ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_EXT_BUF_GROUP, decoder->frm_grp);
            if (ret != MPP_OK) {
                US_MPP_MJPEG_LOG_ERROR("Failed to set external buffer group: %d", ret);
                return US_MPP_ERROR_INIT;
            }
            
            US_MPP_MJPEG_LOG_INFO("Set external buffer group to decoder with %d buffers (%u bytes each)", successful_buffers, buf_size);
        }
        
        // 通知解码器信息处理完成（必须的步骤）
        MPP_RET ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
        if (ret != MPP_OK) {
            US_MPP_MJPEG_LOG_ERROR("Failed to set info change ready: %d", ret);
            return US_MPP_ERROR_DECODE;
        }
        
        return US_MPP_OK;
    }
    
    return US_MPP_OK;
}

static us_mpp_error_e _us_mpp_mjpeg_copy_frame_data(MppFrame mpp_frame, us_frame_s *out_frame) {
    if (!mpp_frame || !out_frame) {
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    MppBuffer buffer = mpp_frame_get_buffer(mpp_frame);
    if (!buffer) {
        US_MPP_MJPEG_LOG_ERROR("No buffer in MPP frame");
        return US_MPP_ERROR_DECODE;
    }
    
    void *src_data = mpp_buffer_get_ptr(buffer);
    size_t src_size = mpp_buffer_get_size(buffer);
    
    if (!src_data || src_size == 0) {
        US_MPP_MJPEG_LOG_ERROR("Invalid buffer data or size");
        return US_MPP_ERROR_DECODE;
    }
    
    // 获取帧信息
    uint32_t width = mpp_frame_get_width(mpp_frame);
    uint32_t height = mpp_frame_get_height(mpp_frame);
    uint32_t hor_stride = mpp_frame_get_hor_stride(mpp_frame);
    uint32_t ver_stride = mpp_frame_get_ver_stride(mpp_frame);
    
    // 计算NV12帧大小
    uint32_t y_size = hor_stride * ver_stride;
    uint32_t uv_size = hor_stride * ver_stride / 2;
    uint32_t total_size = y_size + uv_size;
    
    // 设置输出帧参数
    out_frame->format = V4L2_PIX_FMT_NV12;
    out_frame->width = width;
    out_frame->height = height;
    out_frame->stride = hor_stride;
    out_frame->used = total_size;
    
    // 分配输出缓冲区
    us_frame_realloc_data(out_frame, total_size);
    if (!out_frame->data) {
        US_MPP_MJPEG_LOG_ERROR("Failed to allocate output frame buffer");
        return US_MPP_ERROR_MEMORY;
    }
    
    // 拷贝数据
    memcpy(out_frame->data, src_data, total_size);
    
    US_MPP_MJPEG_LOG_DEBUG("Copied NV12 frame: %ux%u, stride: %u, size: %u", 
                          width, height, hor_stride, total_size);
    
    return US_MPP_OK;
}

// ==== MJPEG解码器公共API ====

us_mpp_error_e us_mpp_mjpeg_decoder_create(us_mpp_processor_s **decoder,
                                          uint32_t max_width, 
                                          uint32_t max_height) {
    if (!decoder) {
        US_MPP_MJPEG_LOG_ERROR("Decoder pointer is NULL");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    *decoder = NULL;
    
    // 分配解码器结构
    us_mpp_processor_s *dec = calloc(1, sizeof(us_mpp_processor_s));
    if (!dec) {
        US_MPP_MJPEG_LOG_ERROR("Failed to allocate decoder structure");
        return US_MPP_ERROR_MEMORY;
    }
    
    // 初始化基础处理器
    us_mpp_error_e err = _us_mpp_processor_init_base(dec, US_MPP_CODEC_MJPEG_DEC);
    if (err != US_MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to initialize base processor: %s", us_mpp_error_string(err));
        free(dec);
        return err;
    }
    
    // 设置解码器参数
    dec->width = max_width;
    dec->height = max_height;
    dec->zero_copy_enabled = true;  // 默认启用零拷贝
    
    // 初始化解码器
    err = _us_mpp_mjpeg_setup_decoder(dec);
    if (err != US_MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to setup MJPEG decoder: %s", us_mpp_error_string(err));
        us_mpp_processor_destroy(dec);
        return err;
    }
    
    // 关键修复：Advanced模式必须预分配buffer（参考mpi_dec_test.c）
    // frame已经在_us_mpp_processor_init_base中初始化
    
    // 预分配frame buffer group和buffer
    uint32_t hor_stride = MPP_ALIGN(max_width, 16);
    uint32_t ver_stride = MPP_ALIGN(max_height, 16); 
    uint32_t buf_size = hor_stride * ver_stride * 4;  // JPEG需要4倍大小
    
    // 修复：使用internal buffer group（更简单可靠的方式）
    MPP_RET ret = mpp_buffer_group_get_internal(&dec->frm_grp, MPP_BUFFER_TYPE_ION);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to get internal buffer group: %d", ret);
        us_mpp_processor_destroy(dec);
        return US_MPP_ERROR_MEMORY;
    }
    
    ret = mpp_buffer_get(dec->frm_grp, &dec->frm_buf, buf_size);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to get frame buffer: %d", ret);
        us_mpp_processor_destroy(dec);
        return US_MPP_ERROR_MEMORY;
    }
    
    // 设置预分配的buffer到frame（Advanced模式关键步骤）
    mpp_frame_set_buffer(dec->frame, dec->frm_buf);
    
    US_MPP_MJPEG_LOG_INFO("Pre-allocated: internal frame buffer %u bytes (%ux%u stride)", 
                         buf_size, hor_stride, ver_stride);
    
    atomic_store(&dec->initialized, true);
    *decoder = dec;
    
    US_MPP_MJPEG_LOG_INFO("MJPEG decoder created successfully (max: %ux%u)", max_width, max_height);
    return US_MPP_OK;
}

us_mpp_error_e us_mpp_mjpeg_decoder_decode(us_mpp_processor_s *decoder,
                                          const us_frame_s *mjpeg_frame,
                                          us_frame_s *nv12_frame) {
    if (!decoder || !mjpeg_frame || !nv12_frame) {
        US_MPP_MJPEG_LOG_ERROR("Invalid parameters");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    if (!atomic_load(&decoder->initialized)) {
        US_MPP_MJPEG_LOG_ERROR("Decoder not initialized");
        return US_MPP_ERROR_NOT_INITIALIZED;
    }
    
    if (!us_mpp_is_format_supported_for_decode(mjpeg_frame->format)) {
        US_MPP_MJPEG_LOG_ERROR("Unsupported input format: %u", mjpeg_frame->format);
        return US_MPP_ERROR_FORMAT_UNSUPPORTED;
    }
    
    if (mjpeg_frame->used == 0 || !mjpeg_frame->data) {
        US_MPP_MJPEG_LOG_ERROR("Empty MJPEG frame data");
        return US_MPP_ERROR_INVALID_PARAM;
    }
    
    us_mpp_error_e result = US_MPP_OK;
    MPP_RET ret = MPP_OK;
    
    pthread_mutex_lock(&decoder->mutex);
    
    if (decoder->should_stop) {
        pthread_mutex_unlock(&decoder->mutex);
        return US_MPP_ERROR_NOT_INITIALIZED;
    }
    
    atomic_store(&decoder->processing, true);
    
    // 验证MJPEG数据有效性（调试修复）
    if (mjpeg_frame->used < 2 || !mjpeg_frame->data) {
        US_MPP_MJPEG_LOG_ERROR("Invalid MJPEG data: size=%zu, data=%p", mjpeg_frame->used, mjpeg_frame->data);
        result = US_MPP_ERROR_INVALID_PARAM;
        goto cleanup;
    }
    
    // 检查JPEG magic bytes (0xFF 0xD8)
    uint8_t *data_ptr = (uint8_t *)mjpeg_frame->data;
    if (data_ptr[0] != 0xFF || data_ptr[1] != 0xD8) {
        US_MPP_MJPEG_LOG_ERROR("Invalid JPEG header: 0x%02X 0x%02X (expected 0xFF 0xD8)", 
                              data_ptr[0], data_ptr[1]);
        result = US_MPP_ERROR_DECODE;
        goto cleanup;
    }
    
    
    // 关键修复：Advanced模式需要用MppBuffer初始化packet（参考mpi_dec_test.c line 286）
    // 确保有输入buffer group
    if (!decoder->pkt_grp) {
        ret = mpp_buffer_group_get_internal(&decoder->pkt_grp, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
        if (ret != MPP_OK) {
            US_MPP_MJPEG_LOG_ERROR("Failed to get input buffer group: %d", ret);
            result = US_MPP_ERROR_MEMORY;
            goto cleanup;
        }
    }
    
    // 先将MJPEG数据拷贝到MppBuffer中
    MppBuffer input_buffer = NULL;
    ret = mpp_buffer_get(decoder->pkt_grp, &input_buffer, mjpeg_frame->used);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to get input buffer: %d", ret);
        result = US_MPP_ERROR_MEMORY;
        goto cleanup;
    }
    
    // 拷贝MJPEG数据到buffer
    void *buffer_ptr = mpp_buffer_get_ptr(input_buffer);
    if (!buffer_ptr) {
        US_MPP_MJPEG_LOG_ERROR("Failed to get buffer pointer");
        mpp_buffer_put(input_buffer);
        result = US_MPP_ERROR_MEMORY;
        goto cleanup;
    }
    memcpy(buffer_ptr, mjpeg_frame->data, mjpeg_frame->used);
    
    // 销毁旧packet并用buffer重新初始化（Advanced模式必需）
    mpp_packet_deinit(&decoder->packet);
    ret = mpp_packet_init_with_buffer(&decoder->packet, input_buffer);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to init packet with buffer: %d", ret);
        mpp_buffer_put(input_buffer);
        result = US_MPP_ERROR_MEMORY;
        goto cleanup;
    }
    
    // 关键修复：设置packet的数据信息（参考mpi_dec_nt_test.c line 94-98）
    void *buffer_data = mpp_buffer_get_ptr(input_buffer);
    size_t buffer_size = mpp_buffer_get_size(input_buffer);
    
    mpp_packet_set_data(decoder->packet, buffer_data);
    mpp_packet_set_size(decoder->packet, buffer_size);
    mpp_packet_set_pos(decoder->packet, buffer_data);
    mpp_packet_set_length(decoder->packet, mjpeg_frame->used);  // 实际数据长度
    mpp_packet_set_buffer(decoder->packet, input_buffer);       // 确保buffer关联正确
    
    // 释放buffer引用（packet已经持有）
    mpp_buffer_put(input_buffer);
    
    // 设置packet状态
    mpp_packet_set_pts(decoder->packet, 0);
    mpp_packet_set_dts(decoder->packet, 0);
    
    
    // 关键修复：使用预分配的frame（Advanced模式，参考mpi_dec_test.c dec_advanced）
    // 不要每次都创建新的frame，使用decoder->frame
    if (!decoder->frame) {
        US_MPP_MJPEG_LOG_ERROR("No pre-allocated frame available");
        result = US_MPP_ERROR_NOT_INITIALIZED;
        goto cleanup;
    }
    
    // 使用meta关联packet和frame（advanced模式必需）
    MppMeta meta = mpp_packet_get_meta(decoder->packet);
    if (meta) {
        mpp_meta_set_frame(meta, KEY_OUTPUT_FRAME, decoder->frame);
    }
    
    
    // 发送数据包进行解码
    ret = decoder->mpi->decode_put_packet(decoder->ctx, decoder->packet);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to put decode packet: %d", ret);
        result = US_MPP_ERROR_DECODE;
        goto cleanup;
    }
    
    // Advanced模式：一次put对应一次get（使用预分配frame）
    MppFrame returned_frame = NULL;
    ret = decoder->mpi->decode_get_frame(decoder->ctx, &returned_frame);
    if (ret != MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("decode_get_frame failed ret %d", ret);
        result = (ret == MPP_ERR_TIMEOUT) ? US_MPP_ERROR_TIMEOUT : US_MPP_ERROR_DECODE;
        goto cleanup;
    }
    
    if (!returned_frame) {
        US_MPP_MJPEG_LOG_ERROR("decode_get_frame returned null frame");
        result = US_MPP_ERROR_DECODE;
        goto cleanup;
    }
    
    // 验证frame引用匹配（advanced模式检查）
    
    // 检查是否是info_change帧
    if (mpp_frame_get_info_change(returned_frame)) {
        US_MPP_MJPEG_LOG_INFO("Received info change frame");
        us_mpp_error_e info_err = _us_mpp_mjpeg_process_info_change(decoder, returned_frame);
        
        if (info_err != US_MPP_OK) {
            US_MPP_MJPEG_LOG_ERROR("Failed to process info change: %s", us_mpp_error_string(info_err));
            result = info_err;
        } else {
            // info_change处理完成，需要重新解码
            result = US_MPP_ERROR_INFO_CHANGE;  // 特殊返回码，调用者应重试
        }
        goto cleanup;
    }
    
    // 检查frame状态
    uint32_t err_info = mpp_frame_get_errinfo(returned_frame);
    uint32_t discard = mpp_frame_get_discard(returned_frame);
    
    if (err_info || discard) {
        US_MPP_MJPEG_LOG_ERROR("Frame with err_info %u discard %u", err_info, discard);
        result = US_MPP_ERROR_DECODE;
        goto cleanup;
    }
    
    if (mpp_frame_get_eos(returned_frame)) {
        result = US_MPP_ERROR_EOS;
        goto cleanup;
    }
    
    // 拷贝解码数据（不需要deinit预分配的frame）
    us_mpp_error_e copy_err = _us_mpp_mjpeg_copy_frame_data(returned_frame, nv12_frame);
    
    if (copy_err != US_MPP_OK) {
        US_MPP_MJPEG_LOG_ERROR("Failed to copy frame data: %s", us_mpp_error_string(copy_err));
        result = copy_err;
    } else {
        result = US_MPP_OK;
    }
    
cleanup:
    decoder->frame_number++;
    pthread_mutex_unlock(&decoder->mutex);
    return result;
}
