#pragma once
/**
 ********************************************************************************************************
 * @file info.hpp
 * @brief 调试UI信息面板（第六窗口 Info）
 * @date 2026-09-26
 *
 * debug模式下以纯文字形式汇总展示车辆运行状态：
 * FSM模式、速度、舵机、AI检测结果
 * 注：OpenCV putText 不支持中文，面板标签统一使用英文
 ********************************************************************************************************
 */

#include <map>
#include <vector>
#include "utils/tools.hpp"
#include "utils/params.hpp"

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
 * @param results AI检测结果（调用处需在锁内拷贝后传入，避免跨线程竞态）
 * @return Mat 信息面板图像(COLSIMAGE x ROWSIMAGE)
 */
inline Mat drawInfo(const shared_ptr<Params> &params,
                    const vector<PredictResult> &results)
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

    // [2] 速度 / 舵机PWM
    string str = "Spd: " + doble2String(params->ctrl.speed, 2) +
                 "  Svo: " + to_string(params->ctrl.servo);
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, white, 1);
    y += lineH;

    // [3] AI检测结果总数
    str = "AI: " + to_string(results.size()) + " targets";
    putText(img, str, Point(5, y), FONT_HERSHEY_SIMPLEX, 0.38, yellow, 1);
    y += lineH;

    // [4] AI各类目标计数（仅显示数量>0的类别，每行最多3项）
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

    return img;
}
