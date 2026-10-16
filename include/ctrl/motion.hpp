#pragma once
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
 * @file motion.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 运动控制器
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>
#include "utils/tools.hpp"
#include "utils/params.hpp"
#include "fsm/park.hpp"
#include <algorithm>

using namespace std;
/**
 * @brief 纯追踪控制器（仅用于非 NORMAL 模式）
 * 
 * 输入：目标点相对于车头方向的夹角 alpha（弧度）和前视距离 ld（米）
 * 输出：前轮转向角（弧度）
 */
class PurePursuit {
private:
    double L_;                // 轴距（米），固定为 0.20
    double max_steer_rad_;    // 最大转向角（弧度），默认 60°（对应舵机极限）

public:
    explicit PurePursuit(double wheelbase = 0.20, double max_steer_deg = 40.0)
        : L_(wheelbase), max_steer_rad_(max_steer_deg * M_PI / 180.0) {}

    /**
     * @brief 计算转向角（弧度）
     * @param alpha  目标点相对于车头方向的夹角（弧度），正值为左转
     * @param ld     目标点到车辆的距离（米），必须 > 0
     * @return 前轮转向角（弧度），正值为左转，范围被限制在 ±max_steer_rad_
     */
    double computeSteering(double alpha, double ld) const {
        if (ld < 1e-6) return 0.0;   // 防止除零
        double delta = atan2(2.0 * L_ * sin(alpha), ld);
        return clamp(delta, -max_steer_rad_, max_steer_rad_);
    }
};


/**
 * @brief 运动控制类
 * 
 * - NORMAL 模式：使用原有的图像偏差 PD 控制（保持兼容）
 * - 其他模式（如 AVOID、COLLECT 等）：使用纯追踪算法
 */
class Motion
{

public:
        void poseControl(shared_ptr<Params> &params, shared_ptr<FsmPark> Park,shared_ptr<FsmBusy> busy) 
        {  
            // ===== 停车场特殊模式：ENTER / EXIT =====
            if (Park && (Park->step == FsmPark::Step::ENTER || Park->step == FsmPark::Step::EXIT))
            {
                static int errorLast = 0;
                int error = params->ctrl.park_err;

                int pwmDiff = (error * params->config.runP1) + (error - errorLast) * params->config.runP2;
                errorLast = error;

                if (params->ctrl.back)
                    params->ctrl.servo = (uint16_t)(PWMSERVOMID + pwmDiff);
                else
                    params->ctrl.servo = (uint16_t)(PWMSERVOMID - pwmDiff);

                // 限幅
                if (params->ctrl.servo > PWMSERVOMAX) params->ctrl.servo = PWMSERVOMAX;
                if (params->ctrl.servo < PWMSERVOMIN) params->ctrl.servo = PWMSERVOMIN;
                return;
            }
             if(params->ctrl.parkleft)
            {
                params->ctrl.servo=PWMSERVOMID-params->config.parkleft;
                return;
            }
            // ===== PARKING 状态 =====
            if (Park && Park->step == FsmPark::Step::PARKING)
            {
                params->ctrl.servo = PWMSERVOMID;
                return;
            }

            // ========== 普通模式：纯追踪 + 微分阻尼 ==========
            // 1. 计算目标转向角（弧度）
            params->ctrl.ld = sqrt(
                pow((params->ctrl.mubiaoni.x - 400) * 0.0027, 2) + 
                pow((499 - params->ctrl.mubiaoni.y) * 0.0027, 2)
            );
            params->ctrl.rad = atan2((params->ctrl.mubiaoni.x - 400), (499 - params->ctrl.mubiaoni.y));

            double steer_rad = pp_controller_.computeSteering(params->ctrl.rad, params->ctrl.ld);
            //cout<<"锥桶"<<params->ctrl.obsDetectFlag<<"  pianyi"<<params->ctrl.pianyi<<params->config.runP2<<endl;
            //cout<<"rad:"<<steer_rad<<endl;
            // 2. 弧度 → PWM 映射
            constexpr double MAX_STEER_RAD = 40.0 * M_PI / 180.0; // 1.0472 rad
            double pwm_range_per_side = static_cast<double>(PWMSERVOMID - PWMSERVOMIN);
            double pwm_scale = pwm_range_per_side / MAX_STEER_RAD;

            // 3. 计算基础 PWM 偏差（P 项）
            int pwmDiff = static_cast<int>(steer_rad * pwm_scale);
            int lastpwmDiff=0;
            // // ---- 新增：微分阻尼（抑制画龙） ----
            // static double steer_rad_last = 0.0;
            // double d_steer = steer_rad - steer_rad_last;        // 曲率变化率
            // steer_rad_last = steer_rad;
            if (params->mode == FsmMode::BUSY)
              {
                pwmDiff=params->config.turnP * pwmDiff;
              }
            // 避障偏移：施加在 turnP 缩放之后，保证施工区内绕行力度不衰减（与NORMAL模式一致）
            if(params->ctrl.obsDetectFlag)
            {
                pwmDiff += static_cast<int>(params->ctrl.pianyi * pwm_scale);
            }
             // cout<<"params->ctrl.parkleft"<<params->ctrl.parkleft<<endl;

        //     if (params->mode == FsmMode::PARK)
        //      {

        //    pwmDiff = lastpwmDiff*0.5+pwmDiff*0.5;
        //    lastpwmDiff=pwmDiff;
        //     // 偏差超出范围，拒绝新值，返回上次有效
             
        //      }

            if(params->ctrl.parkleft)
            {
                pwmDiff=params->config.parkleft;
            }


            // 4. 计算最终舵机 PWM
            int servo_pwm = PWMSERVOMID - pwmDiff;

            // 5. 限幅保护
            if (servo_pwm > PWMSERVOMAX) servo_pwm = PWMSERVOMAX;
            if (servo_pwm < PWMSERVOMIN) servo_pwm = PWMSERVOMIN;

            params->ctrl.servo = static_cast<uint16_t>(servo_pwm);
        }
    /**
     * @brief 变加速控制
     *
     * @param params
     */
    void speedControl(shared_ptr<Params> &params)
    {
        if (params->ctrl.stop) // 停车
        {
            params->ctrl.speed = 0.0;
            return;
        }

        if (params->ctrl.crossStop) // 斑马线停车
        {
            params->ctrl.speed = 0.0;
            return;
        }

        if (params->ctrl.back) // 倒车(停车场)
        {
            params->ctrl.speed = -params->config.velPark;
            return;
        }
         else if (params->ctrl.slow) // 减速区速度
        {
            params->ctrl.speed = params->config.velSlow;
            return;
        }
        else if (params->mode == FsmMode::PARK) // 停车场速度
        {
            params->ctrl.speed = params->config.velPark;
            return;
        }
        else if (params->mode == FsmMode::CROSS) // 斑马线速度
        {
            params->ctrl.speed = params->config.velCross;
            return;
        }
        else if (params->mode == FsmMode::BUSY) // 施工区速度
        {
            params->ctrl.speed = params->config.velBusy;
            return;
        }
        else if (params->mode == FsmMode::YFORK) // Y型岔路口速度
        {
            params->ctrl.speed = params->config.velYfork;
            return;
        }
       
        else
        {
            // NORMAL 模式：固定速度 velHigh（useStanley 已带 ±20% 迟滞，与预瞄距离同步）
            params->ctrl.speed = params->config.velHigh;
        }
    }

        void clampServo(shared_ptr<Params> &params)
        {
            if (params->ctrl.servo > PWMSERVOMAX)
                params->ctrl.servo = PWMSERVOMAX;
            else if (params->ctrl.servo < PWMSERVOMIN)
                params->ctrl.servo = PWMSERVOMIN;
        }
    /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    void outlineCheck(shared_ptr<Params> &params)
    {
        // 停车场模式下车辆主动脱离赛道线，不检测
        if (params->ctrl.parking)
        {
            countRes = 0;
            outline = false;
            return;
        }

        // 出库后冷却期，给车辆时间重新捕获赛道线
        if (params->ctrl.outlineCooldown > 0)
        {
            params->ctrl.outlineCooldown--;
            countRes = 0;
            outline = false;
            return;
        }

        if (outline) // 已出现
        {
            params->ctrl.stop = true; // 停车
            countRes++;
            if (countRes > 15)
            {
                std::cout << "-----> [Stop] Game over, system exit!!! <-----" << std::endl;
                std::_Exit(0); // 直接退出，跳过静态析构避免ONNX崩溃
            }
        }
        else // 出线检测
        {
            if (params->track->pointsEdgeLeft.size() < 30 &&
                params->track->pointsEdgeRight.size() < 30) // 防止车辆冲出赛道
            {
                countRes++;
                countOut = 0;
                if (countRes > 30)
                    outline = true;
            }
            else
            {
                countRes = 0; // 边线恢复立即归零，避免跨圈/跨路段累加
                countOut = 0;
            }
        }
    }

private:
    PurePursuit pp_controller_;
    int countRes = 0;
    int countOut = 0;
    bool outline = false; // 出线标志
};
