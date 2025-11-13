#ifndef DRVASSIST_H
#define DRVASSIST_H

#include <stdbool.h>

/* ▼ 構造体で状態を一元管理 */
typedef struct {
    float speed_kmh;     /* 速度 */
    float front_dist_m;  /* 前方距離 */
    float lane_offset_m; /* レーン中心からのズレ（±） */
    float long_g;        /* 縦G（+加速/-減速） */
} SensorFrame;

/* ▼ 構造体で状態を一元管理（フラグ類） */
typedef struct {
    bool  isRecording;
    bool  laneAlert;
    bool  fcwAlert;
} DashState;

/* ▼ 構造体で状態を一元管理（キャリブレーション値） */
typedef struct {
    /* 記録フラグのヒステリ */
    float rec_on_speed_kmh;
    float rec_off_speed_kmh;
    float rec_on_abs_g;
    float rec_off_abs_g;

    /* レーン逸脱 */
    float lane_on_offset_m;
    float lane_off_offset_m;
    float lane_on_speed_kmh;
    float lane_off_speed_kmh;

    /* 前方衝突FCW */
    float fcw_on_dist_m;
    float fcw_off_dist_m;
    float fcw_on_speed_kmh;
} AssistCalib;

SensorFrame read_sensors(void);
bool        write_video_frame(bool recording);

/* ▼ フラグ切替は共通関数 toggle_hysteresis_bool() に集約 */
/* 汎用フラグ切替 */
bool toggle_hysteresis_bool(bool current, bool on_cond, bool off_cond);

void update_recording(const SensorFrame* s, DashState* st, const AssistCalib* c);
void update_lane_alert(const SensorFrame* s, DashState* st, const AssistCalib* c);
void update_fcw_alert(const SensorFrame* s, DashState* st, const AssistCalib* c);

void process_once(DashState* st, const AssistCalib* c);

#endif
