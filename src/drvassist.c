#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "drvassist.h"

/* =========================================================
 * センサー値を取得する関数群と状態更新関数群
 * ========================================================= */

/* ---------------------------------------------------------
 * 【関数名】 read_sensors
 * 【概要】   擬似的に車両センサー値を生成する。
 * 【役割】   実際のセンサー入力を模擬し、速度・距離・レーンずれ・G値を返す。
 * 【戻り値】 SensorFrame構造体（各種センサー値を含む）
 * --------------------------------------------------------- */
SensorFrame read_sensors(void)
{
    SensorFrame s;
    s.speed_kmh     = 40.0f + (rand() % 50);            // 車速（40〜90km/hの範囲でランダム）
    s.front_dist_m  = 5.0f + (rand() % 50);             // 前方車両までの距離（5〜55m）
    s.lane_offset_m = ((rand()%200)-100)/100.0f;        // レーン中心からのズレ（-1.0〜+1.0m）
    s.long_g        = ((rand()%40)-20)/10.0f;           // 縦加速度（-2.0〜+2.0G）
    return s;
}

/* ---------------------------------------------------------
 * 【関数名】 toggle_hysteresis_bool
 * 【概要】   フラグのON/OFFをヒステリシス付きで切り替える。
 * 【引数】   current: 現在の状態（true=ON / false=OFF）
 *             on_cond: ONにする条件（trueのときONへ）
 *             off_cond: OFFにする条件（trueのときOFFへ）
 * 【戻り値】 新しい状態（ON/OFF）
 * --------------------------------------------------------- */
bool toggle_hysteresis_bool(bool current, bool on_cond, bool off_cond)
{
    if (!current && on_cond)  return true;   // OFF→ON 条件成立
    if ( current && off_cond) return false;  // ON→OFF 条件成立
    return current;                          // 状態維持
}

/* ---------------------------------------------------------
 * 【関数名】 write_video_frame
 * 【概要】   録画フレームの出力を行う（ダミー実装）
 * 【引数】   recording: 現在の録画フラグ（trueなら録画中）
 * 【戻り値】 成功(true)を返す（常に成功する仮実装）
 * --------------------------------------------------------- */
bool write_video_frame(bool recording)
{
    (void)recording;
    return true; /* 例では常に成功 */
}

/* ---------------------------------------------------------
 * 【関数名】 update_recording
 * 【概要】   車速とG値に応じて録画フラグを更新する。
 * 【引数】   s : センサー値
 *             st: 現在の状態（RECフラグ含む）
 *             c : 閾値設定（キャリブレーション情報）
 * 【処理内容】
 *   - 速度 > rec_on_speed_kmh または |G| > rec_on_abs_g → 録画ON
 *   - 速度 < rec_off_speed_kmh かつ |G| < rec_off_abs_g → 録画OFF
 * --------------------------------------------------------- */
void update_recording(const SensorFrame* s, DashState* st, const AssistCalib* c)
{
    bool on_cond  = (s->speed_kmh > c->rec_on_speed_kmh) || (fabsf(s->long_g) > c->rec_on_abs_g);
    bool off_cond = (s->speed_kmh < c->rec_off_speed_kmh) && (fabsf(s->long_g) < c->rec_off_abs_g);

    st->isRecording = toggle_hysteresis_bool(st->isRecording, on_cond, off_cond);
}

/* ---------------------------------------------------------
 * 【関数名】 update_lane_alert
 * 【概要】   レーン逸脱の状況を判定し、警報フラグを更新。
 * 【引数】   s : センサー値
 *             st: 現在の状態（LANEフラグ含む）
 *             c : 閾値設定
 * 【処理内容】
 *   - |lane_offset| > lane_on_offset_m かつ 速度 > lane_on_speed_kmh → ON
 *   - |lane_offset| < lane_off_offset_m または 速度 < lane_off_speed_kmh → OFF
 * --------------------------------------------------------- */
void update_lane_alert(const SensorFrame* s, DashState* st, const AssistCalib* c)
{
    bool on_cond  = (fabsf(s->lane_offset_m) > c->lane_on_offset_m) && (s->speed_kmh > c->lane_on_speed_kmh);
    bool off_cond = (fabsf(s->lane_offset_m) < c->lane_off_offset_m) || (s->speed_kmh < c->lane_off_speed_kmh);

    st->laneAlert = toggle_hysteresis_bool(st->laneAlert, on_cond, off_cond);
}

/* ---------------------------------------------------------
 * 【関数名】 update_fcw_alert
 * 【概要】   前方車両との距離に基づいて衝突警報を出す。
 * 【引数】   s : センサー値
 *             st: 現在の状態（FCWフラグ含む）
 *             c : 閾値設定
 * 【処理内容】
 *   - 前方距離 < fcw_on_dist_m かつ 速度 > fcw_on_speed_kmh → ON
 *   - 前方距離 > fcw_off_dist_m → OFF
 * --------------------------------------------------------- */
void update_fcw_alert(const SensorFrame* s, DashState* st, const AssistCalib* c)
{
    bool on_cond  = (s->front_dist_m < c->fcw_on_dist_m) && (s->speed_kmh > c->fcw_on_speed_kmh);
    bool off_cond = (s->front_dist_m > c->fcw_off_dist_m);

    st->fcwAlert = toggle_hysteresis_bool(st->fcwAlert, on_cond, off_cond);
}

/* ---------------------------------------------------------
 * 【関数名】 process_once
 * 【概要】   1サイクル分の制御処理をまとめて実行する。
 * 【役割】
 *   1. センサー値を読み取る
 *   2. 各機能の判定（録画・レーン逸脱・衝突警報）
 *   3. 出力処理（録画実行）
 * --------------------------------------------------------- */
void process_once(DashState* st, const AssistCalib* c)
{
    /* --- センサー読み取り（入力） --- */
    SensorFrame s = read_sensors();

    /* --- 各判定をモジュール化して呼び出し（判断） --- */
    update_recording(&s, st, c);   // 記録ON/OFF判定
    update_lane_alert(&s, st, c);  // レーン逸脱判定
    update_fcw_alert(&s, st, c);   // 前方衝突判定

    /* --- 出力処理 --- */
    bool ok = write_video_frame(st->isRecording);
    if (!ok) {
        fprintf(stderr, "[ERR] write frame failed\n");
    }

    /* --- 結果表示（デバッグ用） --- */
    printf("v=%.1fkm/h dist=%.1fm lane=%.2fm g=%.2f | REC=%d LANE=%d FCW=%d\n",
           s.speed_kmh, s.front_dist_m, s.lane_offset_m, s.long_g,
           st->isRecording, st->laneAlert, st->fcwAlert);
}
