#ifndef US_MPP_ENCODER_H
#define US_MPP_ENCODER_H

#include <stdatomic.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <linux/videodev2.h>

#include "rk_mpi.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "../../../libs/frame.h"

#define US_MPP_TIMEOUT_MS 100

// MPP对齐宏（参考MPP源码）
#ifndef MPP_ALIGN
#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// MPP编码器类型
typedef enum {
    US_MPP_CODEC_MJPEG_DEC = 0,  // MJPEG解码器
    US_MPP_CODEC_H264_ENC,       // H264编码器
    US_MPP_CODEC_H265_ENC,       // H265编码器 (预留)
} us_mpp_codec_type_e;

// MPP编码器错误码
typedef enum {
    US_MPP_OK = 0,
    US_MPP_ERROR_INVALID_PARAM = -1,
    US_MPP_ERROR_MEMORY = -2,
    US_MPP_ERROR_INIT = -3,
    US_MPP_ERROR_ENCODE = -4,
    US_MPP_ERROR_DECODE = -5,
    US_MPP_ERROR_FORMAT_UNSUPPORTED = -6,
    US_MPP_ERROR_DEVICE_NOT_FOUND = -7,
    US_MPP_ERROR_DEVICE_BUSY = -8,
    US_MPP_ERROR_HARDWARE_FAILURE = -9,
    US_MPP_ERROR_NOT_INITIALIZED = -10,
    US_MPP_ERROR_BUFFER_OVERFLOW = -11,
    US_MPP_ERROR_TIMEOUT = -12,
    US_MPP_ERROR_INFO_CHANGE = -13,  // Info change事件，需要重试
    US_MPP_ERROR_EOS = -14,          // 流结束
} us_mpp_error_e;

// MPP编码器统计信息
typedef struct {
    uint64_t frames_processed;
    uint64_t bytes_input;
    uint64_t bytes_output;
    uint64_t processing_errors;
    double avg_processing_time_ms;
    double total_processing_time_ms;
    double current_fps;
    uint64_t last_stats_update;
    
    // 解码器专用统计
    uint64_t frames_decoded;
    uint64_t decode_errors;
    
    // 编码器专用统计
    uint64_t frames_encoded;
    uint64_t encode_errors;
    uint32_t keyframes_generated;
} us_mpp_stats_s;

// 直接包含MPP头文件和V4L2格式定义
#include "rk_mpi.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include <linux/videodev2.h>

// 定义缓冲区数量常量
#define US_MPP_DECODER_BUFFER_COUNT 8
#define US_MPP_ENCODER_BUFFER_COUNT 8

// MPP缓冲区管理
typedef struct {
    MppBufferGroup buffer_group;
    MppBuffer *input_buffers;
    MppBuffer *output_buffers;
    uint32_t input_buffer_count;
    uint32_t output_buffer_count;
    uint32_t buffer_size;
    pthread_mutex_t mutex;
} us_mpp_buffer_mgr_s;

// MPP编码器/解码器结构体
typedef struct {
    // MPP核心资源
    MppCtx ctx;
    MppApi *mpi;
    
    // 缓冲区管理
    us_mpp_buffer_mgr_s buffer_mgr;
    MppBufferGroup frm_grp;     // 帧缓冲区组（MJPEG解码专用）
    MppBuffer frm_buf;          // 帧缓冲区（MJPEG解码专用）
    MppBufferGroup pkt_grp;     // 包缓冲区组（MJPEG输入专用）
    MppBuffer pkt_buf;          // 输出包缓冲区（H264编码专用）
    MppPacket packet;
    MppFrame frame;
    
    // 编码器配置 (仅编码器使用)
    MppEncCfg enc_cfg;
    
    // 基本参数
    us_mpp_codec_type_e codec_type;
    uint32_t width;
    uint32_t height;
    uint32_t hor_stride;
    uint32_t ver_stride;
    uint32_t bitrate_bps;
    uint32_t fps_num;
    uint32_t fps_den;
    uint32_t gop_size;
    
    // 编码器专用参数
    uint32_t profile;        // H264/H265 profile
    uint32_t level;          // H264/H265 level  
    uint32_t rc_mode;        // 码率控制模式: 0=VBR, 1=CBR
    uint32_t qp_init;        // 初始QP值
    uint32_t qp_min;         // 最小QP值
    uint32_t qp_max;         // 最大QP值
    
    // 状态管理
    atomic_bool initialized;
    atomic_bool processing;
    volatile bool should_stop;
    
    // 错误处理
    us_mpp_error_e last_error;
    char last_error_msg[256];
    uint32_t consecutive_errors;
    uint32_t max_consecutive_errors;
    
    // 统计信息
    us_mpp_stats_s stats;
    
    // 性能优化
    bool zero_copy_enabled;
    bool parallel_processing;
    
    // 线程安全
    pthread_mutex_t mutex;
    bool mutex_initialized;
    
    // 帧计数器
    uint64_t frame_number;
    uint64_t pts_base;
    
    // 调试信息
    uint32_t debug_level;
    char debug_prefix[64];
} us_mpp_processor_s;

// ==== 主要API函数 ====

// MJPEG解码器API
us_mpp_error_e us_mpp_mjpeg_decoder_create(us_mpp_processor_s **decoder,
                                          uint32_t max_width, 
                                          uint32_t max_height);

us_mpp_error_e us_mpp_mjpeg_decoder_decode(us_mpp_processor_s *decoder,
                                          const us_frame_s *mjpeg_frame,
                                          us_frame_s *nv12_frame);

// H264编码器API  
us_mpp_error_e us_mpp_h264_encoder_create(us_mpp_processor_s **encoder,
                                         uint32_t width, uint32_t height,
                                         uint32_t bitrate_kbps, uint32_t gop_size,
                                         uint32_t fps_num, uint32_t fps_den);

us_mpp_error_e us_mpp_h264_encoder_encode(us_mpp_processor_s *encoder,
                                         const us_frame_s *nv12_frame,
                                         us_frame_s *h264_frame,
                                         bool force_key);

// 通用API
void us_mpp_processor_destroy(us_mpp_processor_s *processor);
us_mpp_error_e us_mpp_processor_reset(us_mpp_processor_s *processor);
us_mpp_error_e us_mpp_processor_get_stats(us_mpp_processor_s *processor, us_mpp_stats_s *stats);

// ==== 辅助函数 ====
const char* us_mpp_error_string(us_mpp_error_e error);
const char* us_mpp_codec_type_string(us_mpp_codec_type_e type);
bool us_mpp_is_format_supported_for_decode(uint32_t format);
bool us_mpp_is_format_supported_for_encode(uint32_t format);

// ==== 配置和调优 ====
us_mpp_error_e us_mpp_processor_set_debug_level(us_mpp_processor_s *processor, uint32_t level);
us_mpp_error_e us_mpp_processor_enable_zero_copy(us_mpp_processor_s *processor, bool enable);
us_mpp_error_e us_mpp_processor_enable_parallel(us_mpp_processor_s *processor, bool enable);

// H264编码器高级配置
us_mpp_error_e us_mpp_h264_encoder_set_profile(us_mpp_processor_s *encoder, uint32_t profile);
us_mpp_error_e us_mpp_h264_encoder_set_rc_mode(us_mpp_processor_s *encoder, uint32_t rc_mode);
us_mpp_error_e us_mpp_h264_encoder_set_qp_range(us_mpp_processor_s *encoder, uint32_t qp_min, uint32_t qp_max);

// ==== 一体化编解码API ====

// 高级API：MJPEG直接转H264 (内部管理NV12缓冲区)
typedef struct {
    us_mpp_processor_s *decoder;  // MJPEG解码器
    us_mpp_processor_s *encoder;  // H264编码器
    us_frame_s *nv12_buffer;      // 中间NV12缓冲区
    pthread_mutex_t mutex;
    bool initialized;
    us_mpp_stats_s combined_stats;
} us_mpp_transcoder_s;

us_mpp_error_e us_mpp_transcoder_create(us_mpp_transcoder_s **transcoder,
                                       uint32_t max_width, uint32_t max_height,
                                       uint32_t bitrate_kbps, uint32_t gop_size,
                                       uint32_t fps_num, uint32_t fps_den);

us_mpp_error_e us_mpp_transcoder_process(us_mpp_transcoder_s *transcoder,
                                        const us_frame_s *mjpeg_frame,
                                        us_frame_s *h264_frame,
                                        bool force_key);

void us_mpp_transcoder_destroy(us_mpp_transcoder_s *transcoder);
us_mpp_error_e us_mpp_transcoder_get_stats(us_mpp_transcoder_s *transcoder, us_mpp_stats_s *stats);

// ==== 内部辅助函数声明 ====
us_mpp_error_e _us_mpp_processor_init_base(us_mpp_processor_s *proc, us_mpp_codec_type_e type);
us_mpp_error_e _us_mpp_init_buffer_manager(us_mpp_buffer_mgr_s *mgr, uint32_t buffer_count, uint32_t buffer_size);
void _us_mpp_update_stats(us_mpp_processor_s *proc, uint64_t process_time_us, bool success, bool is_encode);
uint64_t _us_mpp_get_time_us(void);
uint32_t _us_mpp_calc_frame_size(uint32_t width, uint32_t height, MppFrameFormat fmt);

#ifdef __cplusplus
}
#endif

#endif // US_MPP_ENCODER_H
