#include <criterion/criterion.h>
#include <criterion/logging.h>
#include "drvassist.h"

/* ---- ログ用ヘルパ ---- */
static void log_state(const char* label, const DashState* st) {
    cr_log_info("%s => REC=%d LANE=%d FCW=%d\n",
                label, st->isRecording, st->laneAlert, st->fcwAlert);
}

/* ---- 共通の初期化 ---- */
static void init_state(DashState* st) {
    st->isRecording = false;
    st->laneAlert   = false;
    st->fcwAlert    = false;
}

/* ==============================
 * 個別関数: toggle_hysteresis_bool
 * ============================== */
Test(toggle_hysteresis_bool, transitions) {
    // 初期OFF -> ON条件を満たす
    cr_assert(toggle_hysteresis_bool(false, true,  false) == true);

    // ON -> OFF条件を満たす
    cr_assert(toggle_hysteresis_bool(true,  false, true ) == false);

    // ON -> OFF条件を満たさない → 維持
    cr_assert(toggle_hysteresis_bool(true,  false, false) == true);

    // OFF -> ON条件を満たさない → 維持
    cr_assert(toggle_hysteresis_bool(false, false, false) == false);
}

/* ==============================
 * 記録フラグ: update_recording
 * しきい値はユーザー例に合わせる
 *  rec_on_speed=60, rec_off_speed=50
 *  rec_on_abs_g=0.8, rec_off_abs_g=0.3
 * ============================== */
Test(update_recording, speed_and_g_hysteresis) {
    DashState st; init_state(&st);
    AssistCalib c = {
        .rec_on_speed_kmh  = 60.0f,
        .rec_off_speed_kmh = 50.0f,
        .rec_on_abs_g      = 0.8f,
        .rec_off_abs_g     = 0.3f
    };
    SensorFrame s = {0};

    // 低速でGも低 → OFFのまま
    s.speed_kmh = 40; s.long_g = 0.1f;
    update_recording(&s, &st, &c);
    cr_assert(st.isRecording == false);

    // 高速になった → ON
    s.speed_kmh = 65; s.long_g = 0.1f;
    update_recording(&s, &st, &c);
    cr_assert(st.isRecording == true);

    // 速度落ちたがまだ50以上 → 維持
    s.speed_kmh = 55; s.long_g = 0.1f;
    update_recording(&s, &st, &c);
    cr_assert(st.isRecording == true);

    // 速度40 & G小 → OFF
    s.speed_kmh = 40; s.long_g = 0.1f;
    update_recording(&s, &st, &c);
    cr_assert(st.isRecording == false);

    log_state("update_recording", &st);
}

/* ==============================
 * レーン逸脱: update_lane_alert
 * lane_on_offset=0.5, lane_off_offset=0.2
 * lane_on_speed=50,   lane_off_speed=40
 * ============================== */
Test(update_lane_alert, lane_hysteresis) {
    DashState st; init_state(&st);
    AssistCalib c = {
        .lane_on_offset_m   = 0.5f,
        .lane_off_offset_m  = 0.2f,
        .lane_on_speed_kmh  = 50.0f,
        .lane_off_speed_kmh = 40.0f
    };
    SensorFrame s = {0};

    // 低速 & 中心付近 → OFF
    s.speed_kmh = 30; s.lane_offset_m = 0.1f;
    update_lane_alert(&s, &st, &c);
    cr_assert(st.laneAlert == false);

    // 速度アップ & ズレ大きい → ON
    s.speed_kmh = 60; s.lane_offset_m = 0.6f;
    update_lane_alert(&s, &st, &c);
    cr_assert(st.laneAlert == true);

    // ズレが減ったがまだOFF条件未満 → 維持
    s.lane_offset_m = 0.3f; s.speed_kmh = 45;
    update_lane_alert(&s, &st, &c);
    cr_assert(st.laneAlert == true);

    // 中心に戻り速度も低下 → OFF
    s.lane_offset_m = 0.1f; s.speed_kmh = 35;
    update_lane_alert(&s, &st, &c);
    cr_assert(st.laneAlert == false);

    log_state("update_lane_alert", &st);
}

/* ==============================
 * 前方衝突警報: update_fcw_alert
 * fcw_on_dist=10, fcw_off_dist=20, fcw_on_speed=40
 * ============================== */
Test(update_fcw_alert, fcw_hysteresis) {
    DashState st; init_state(&st);
    AssistCalib c = {
        .fcw_on_dist_m    = 10.0f,
        .fcw_off_dist_m   = 20.0f,
        .fcw_on_speed_kmh = 40.0f
    };
    SensorFrame s = {0};

    // 距離遠い → OFF
    s.front_dist_m = 30; s.speed_kmh = 60;
    update_fcw_alert(&s, &st, &c);
    cr_assert(st.fcwAlert == false);

    // 急接近 → ON
    s.front_dist_m = 8; s.speed_kmh = 50;
    update_fcw_alert(&s, &st, &c);
    cr_assert(st.fcwAlert == true);

    // まだ近い → 維持
    s.front_dist_m = 15;
    update_fcw_alert(&s, &st, &c);
    cr_assert(st.fcwAlert == true);

    // 離れた → OFF
    s.front_dist_m = 25;
    update_fcw_alert(&s, &st, &c);
    cr_assert(st.fcwAlert == false);

    log_state("update_fcw_alert", &st);
}
