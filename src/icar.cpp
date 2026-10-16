/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file icar.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "icar.hpp"

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    Icar icar;

    // ============ 固定 30FPS 控制周期调度 ============
    // 采用绝对截止时刻（deadline）推进策略：每一帧在上一帧截止时刻基础上
    // 累加固定周期，使平均周期严格等于 1/30 s，避免 sleep_for 相对方式因单帧
    // 处理耗时抖动（路径规划/速度控制/转向计算波动）导致的周期累积漂移。
    // 若某帧处理耗时超过周期（错过 deadline），则立即重新对齐到当前时刻（丢帧），
    // 避免“死亡螺旋”使后续周期进一步恶化，从而在不同工况下维持稳定 30FPS 输出。
    constexpr double CONTROL_FPS = 30.0;
    const auto controlPeriod = std::chrono::duration<double, std::milli>(1000.0 / CONTROL_FPS); // 33.333...ms
    auto nextWake = std::chrono::steady_clock::now();

    while (1)
    {
        auto timeStart = std::chrono::high_resolution_clock::now();
        icar.running(); // 系统主线程（含路径规划/速度控制/转向控制，均按 30FPS 周期执行）

        // 计算处理耗时，便于监控是否超周期
        auto timeEnd = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart);
         //printf(">> FrameTime: %ldms | %.2ffps \n", elapsed.count(), 1000.0 / elapsed.count());

        // 推进到下一帧的绝对截止时刻并精确休眠
        nextWake += std::chrono::duration_cast<std::chrono::steady_clock::duration>(controlPeriod);
        auto now = std::chrono::steady_clock::now();
        if (now < nextWake)
            std::this_thread::sleep_until(nextWake);
        else
            nextWake = now; // 错过截止时刻，重新同步，保证后续周期稳定
    }

    return 0;
}