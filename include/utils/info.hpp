#pragma once
/**
 ********************************************************************************************************
 * @file info.hpp
 * @brief 调试UI信息面板（第六窗口 Info）
 * @date 2026-09-26
 *
 * debug模式下以纯文字形式汇总展示车辆运行状态：
 * FSM模式、帧号、速度、舵机、串口数据、AI检测结果、报警状态、控制目标点等
 * 注：OpenCV putText 不支持中文，面板标签统一使用英文
 ********************************************************************************************************
 */

#include <map>
#include <vector>
#include "utils/tools.hpp"
#include "utils/params.hpp"
#include "com/uart.hpp"

using namespace std;
using namespace cv;

/**
 * @brief FsmMode枚举转字符串
 *
 * @param mode 状态机模式
 * @return string 模式名称
 */
inline string fsmModeToStr(FsmMode mode)
{
    switch (mode)
    {
    case FsmMode::NORMAL:   return "NORMAL";
    case FsmMode::FORK:     return "FORK";
    case FsmMode::PARK:     return "PARK";
    case FsmMode::BUSY:     return "BUSY";
    case FsmMode::SLOW:     return "SLOW";
    case FsmMode::FINE:     return "FINE";
    case FsmMode::STOP:     return "STOP";
    case FsmMode::CROSS:    return "CROSS";
    case FsmMode::YFORK:    return "YFORK";
    case FsmMode::OBSTACLE: return "OBSTACLE";
    default:                return "UNKNOWN";
    }
}

/**
 * @brief 绘制Info信息面板（黑底白字）
 *
 * @param params 车辆状态参数
 * @param uart 串口通信（超声波距离/陀螺仪角度）
 * @param results AI检测结果（调用处需在锁内拷贝后传入，避免跨线程竞态）
 * @param frame 当前帧号
 * @return Mat 信息面板图像(COLSIMAGE x ROWSIMAGE)
 */
inline Mat drawInfo(const shared_ptr<Params> &params,
                    const shared_ptr<Uart> &uart,
                    const vector<PredictResult> &results,
                    int frame)
{
    Mat img = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3);
    if (!params)
        return img;

    const int lineH = 18;               // 行高
    int y = 20;                         // 起始纵坐标
    const Scalar white(255, 255, 255);
    const Scalar green(0, 255, 0);
    const Scalar yellow(0, 255, 255);

    // [1] FSM模式（绿色高亮）
    putText(img, "Mode: " + fsmModeToStr(params->mode), Point(5, y),
            FONT_HERSHEY_SIMPLEX, 0.45, green, 1);
    y += lineH;

    // [2] 帧号 / 速度 / 舵机PWM
    string str = "Frame: " + to_string(frame) +
                 "  Spd: " + doble2String(params->ctrl.speed, 2) +
                 "  Svo: " + to_string(params->ctrl.servo);
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
    y += lineH;

    // [3] 串口数据：超声波距离 / 陀螺仪角度
    int dist = uart ? uart->Distance.load() : 0;
    int gyro = uart ? uart->gyro_data.load() : 0;
    str = "Dist: " + to_string(dist) + "  Gyro: " + to_string(gyro);
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
    y += lineH;

    // [4] AI检测结果总数
    str = "AI: " + to_string(results.size()) + " targets";
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, yellow, 1);
    y += lineH;

    // [5] AI各类目标计数（仅显示数量>0的类别，每行最多3项）
    {
        map<string, int> cnt;
        for (const auto &r : results)
            cnt[r.label]++;
        int col = 0;
        str.clear();
        for (const auto &kv : cnt)
        {
            str += kv.first + ":" + to_string(kv.second) + "  ";
            if (++col % 3 == 0)
            {
                putText(img, str, Point(15, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
                y += lineH;
                str.clear();
            }
        }
        if (!str.empty())
        {
            putText(img, str, Point(15, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
            y += lineH;
        }
    }

    // [6] 报警状态：减速计数
    str = "Dec:" + to_string(params->alertDecelCount);
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
    y += lineH;

    // [7] 障碍物避障状态：检测标志 / 偏移量
    str = "Obs:" + string(params->ctrl.obsDetectFlag ? "Y" : "N") +
          " Pianyi: " + doble2String(params->ctrl.pianyi, 3);
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
    y += lineH;

    // [8] 控制目标点（IPM坐标）
    str = "Anchor: (" + to_string((int)params->ctrl.anchorIPM.x) + "," +
          to_string((int)params->ctrl.anchorIPM.y) + ")";
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
    y += lineH;

    // [9] 其他：目标车库 / Stanley开关 / 道闸缓存帧龄
    str = "Spot:" + to_string(params->config.parkSpot) +
          " Stanley:" + string(params->ctrl.useStanley ? "ON" : "OFF") +
          " Gate:" + to_string(params->gateCacheAge);
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
    y += lineH;

    return img;
}
