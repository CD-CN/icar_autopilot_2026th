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
 * @file park.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停车场控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/park.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmPark::FsmPark(std::shared_ptr<Params> par, std::shared_ptr<Uart> uart)
    : FSMState(FsmMode::PARK, par)
{
    sensor = uart;
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmPark::~FsmPark()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmPark::getMode()
{
    if (step == Step::NONE || !params->config.park)
    {
        params->ctrl.parking = false;
        return FsmMode::NORMAL;
    }
    else
    {
        params->ctrl.parking = true;
        return FsmMode::PARK;
    }
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmPark::run(Mat &img)
{
    if (!params->config.park) // 该模式未启用
        return;

    if (step != Step::NONE)
        params->track->spurroad.clear(); // 停车场活动期间清除岔路红点，防止yfork误检

    // 已删除 stopping = false;  // 保持停车标志持续有效

    speedUp++;
    if (speedUp > 1000) // 出库缓加速计时
        speedUp = 1000;


    switch (step)
    {
    case Step::NONE: // AI未识别
    {
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_PARK &&
                params->results[i].height < 100 && params->results[i].width < 90 &&
                params->results[i].x > COLSIMAGE / 2) // AI标志：停车场
                {
                cout<<"park底部1:"<<params->results[i].height+params->results[i].y<<"parkH:"<<params->results[i].height
                <<"parkW:"<<params->results[i].width<<endl;
                countRes++;
                }
               
        }

        if (countRes >= 2) // 硬编码
        {
            setStep(Step::ENABLE); // 设置停车场新步骤
            std::cout << "[DEBUG] Entered PARK Step::ENABLE at countRes=" << countRes << std::endl;
        }

        if (countRes > 0) // 识别AI标志后开始场次计数
        {
            countSes++;
            if (countSes >= 5) // 硬编码
            {
                countSes = 0;
                countRes = 0;
            }
        }
        break;
    }

    case Step::ENABLE: // 停车场使能
    {
        params->ctrl.parkleft=false;
        countSes++;
        if (!params->ctrl.stop) // 停车等待
            timeout++;
        if (timeout > 200) // 硬编码
            reset();      // 停车场数据复位

        // 检测入库AI标志
        PredictResult fork;
        fork.score = 0;
        fork.y = 0;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_CHOICE) // AI标志：左转直行
            {
                if (params->results[i].y > fork.y && params->results[i].width < 120 && params->results[i].height < 120)
                {
                    cout<<"park底部2:"<<params->results[i].height+params->results[i].y<<"parkH:"<<params->results[i].height
                <<"parkW:"<<params->results[i].width<<endl;
                    fork = params->results[i]; // 搜索最底行的岔路标志
                }
            }
        }
        if (fork.score > 0) // 检测到AI标志
        {
            if ((fork.y + fork.height / 2) > ROWSIMAGE * params->config.parkEnter.enableForkRowRatio) // 硬编码
                countRes++;
            if (countRes > 1) // 硬编码
            {
            setStep(Step::FORKIN); // 设置停车场新步骤
            params->ctrl.parkleft=true;
            }
        }

        // parkSpot为0时穿过停车场不停车（不提前回退，让流程走到FORKIN）
        break;
    }

    case Step::FORKIN: // 入库岔路转向
    {
        timeout++;
         if (findSymbols(params->results, LABEL_GATE)) // 搜索AI标志：道闸
             countRes++;
        
        if (timeout > params->config.parkEnter.forkinTimeout||countRes >1 ) // 使用配置的超时帧数
        {
            spots.reset(); // 车位信息复位
            // 自动标记非目标车位为已占用（parkSpot指定的是目标空车位）
            for (int i = 1; i <= 4; i++)
            {
                if (i != params->config.parkSpot)
                    spots.counter[i - 1] = 5; // counter > 4 即标记为已占用
            }

            countOut = 0;
            params->ctrl.parkleft=false;
            setStep(Step::TRACKIN); // 设置停车场新步骤
        }
        break;
    }

    case Step::TRACKIN: // 入库巡线中
    {
        timeout++;
        countSes++;

        bool gateNear = false;   // 新增：每帧重置，检测当前帧是否有道闸接近

        //[01] 道闸检测
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_GATE)
            {
                int bottom = params->results[i].y + params->results[i].height;
                // 可根据需要调整阈值，原为 params->config.parkEnter.trackinGateStopRatio，默认可能较小
                if (bottom > ROWSIMAGE * params->config.parkEnter.trackinGateStopRatio)
                {
                    gateNear = true;
                    stopping = true;
                    params->ctrl.stop = true;
                    // 可加break优化，但保留继续检测其他标志
                }

                // 出停车场检测（保持不变）
                if (bottom > ROWSIMAGE * params->config.parkEnter.trackinGateExitRatio && spots.checked)
                {
                    countOut++;
                    if (countOut > params->config.parkEnter.trackinGateExitFrames)
                        setStep(Step::TRACKOUT);
                    break;
                }
                timeout = 0;
                countRes = 0;
            }
        }

        // 如果当前帧没有道闸接近，清除停车标志（允许行驶）
        if (!gateNear)
        {
            stopping = false;
            params->ctrl.stop = false;
        }

        //[02] 控制中心重规划
        spots.forks = findParkStation(params->results); // 搜索停车位坐标
        PredictResult resLeft;
        resLeft.score = 0;
        for (int i = 0; i < params->results.size(); i++) // 搜索左转箭头
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120 &&
                params->results[i].score > resLeft.score)
                resLeft = params->results[i];
        }

        bool hasTarget = false;
        PointX targetPoint;

        // 优先级1：左转箭头（出口标志）
        if (resLeft.score > 0)
        {
            float centerX = resLeft.x - resLeft.width / 2;
            float centerY = resLeft.y - resLeft.height / 2;
            targetPoint = PointX(centerY, centerX);
            hasTarget = true;
        }
        // // 优先级2：只有同时看到两个叉子（近处+远处）才转向，避免单一叉子误识别
        // else if (spots.forks.size() >= 2)
        // {
        //     // 使用近处叉子（spots.forks[1]）作为目标
        //     float targetX = spots.forks[1].x + spots.forks[1].width / 2;
        //     float targetY = spots.forks[1].y + spots.forks[1].height / 2;
        //     targetPoint = PointX(targetY, targetX);
        //     hasTarget = true;
        // }
        // 其他情况（0个或1个叉子）：不转向，生成直线

        // 生成路径
        if (hasTarget)
        {
            // ----- 有可靠目标（左转箭头或两个叉子）：生成贝塞尔曲线 -----
            PointX start = PointX(ROWSIMAGE - 20, COLSIMAGE / 2);
            if (countIn < params->config.parkEnter.trackinBezierInitFrames)
            {
                if (params->track->pointsEdgeLeft.size() > ROWSIMAGE / 5 &&
                    params->track->pointsEdgeRight.size() > ROWSIMAGE / 5)
                {
                    start = {
                        (params->track->pointsEdgeLeft[0].x + params->track->pointsEdgeRight[0].x) / 2,
                        (params->track->pointsEdgeLeft[0].y + params->track->pointsEdgeRight[0].y) / 2
                    };
                }
                countIn++;
            }
            PointX end = targetPoint;
            PointX mid = PointX((start.x + end.x) / 2, (start.y + end.y) / 2);
            vector<PointX> repairPoints = {start, mid, end};
            params->ctrl.centerEdge = Bezier(0.02, repairPoints);
            params->ctrl.fitting = true;
            timeout = 0;
            countRes = 0;

            // 出停车场检测（如果是左转箭头且足够近）
            if (resLeft.score > 0)
            {
                float arrowCenterY = resLeft.y + resLeft.height / 2;
                if (arrowCenterY > ROWSIMAGE * params->config.parkEnter.trackinArrowRowRatio)
                    countOut++;
                if (countOut > params->config.parkEnter.trackinArrowExitFrames)
                    setStep(Step::TRACKOUT);
            }
        }


        //[03] 空闲车位检测
        if (spots.forks.size() == 2) // 当AI图像同时检测到两个岔路箭头时判断车位是否空闲
        {
            for (int i = 0; i < 4; i++)
            {
                if (spots.counter[i] > 4)
                    spots.spotEnable[i] = false; // 停车位已占
            }

            float fork0_center = spots.forks[0].y + spots.forks[0].height / 2;
            float fork1_center = spots.forks[1].y + spots.forks[1].height / 2;
            float threshold = ROWSIMAGE * params->config.parkEnter.spotDown; // 使用配置的 spotDown

            if (fork0_center > threshold || fork1_center > threshold)
                spots.countRes++;
            else
                spots.countRes = 0;
            if (spots.countRes > 1) // 开始确认驶入车位号
            {
                spots.checked = true;
                spots.countRes = 0;
            }
        }

 //[04] 入库检测
if (spots.checked && params->config.spot)
{
    // 远处车位 1/4号（左前/右前）→ 第一个叉
    if (spots.spotEnable[0] || spots.spotEnable[3])
    {
        // 如果还未触发延迟准备，且检测到叉子并且位置达到 spotUp 阈值
        if (!delayTriggerReady && spots.forks.size() > 0)
        {
            float forkCenterY = spots.forks[0].y + spots.forks[0].height / 2;
            if (forkCenterY > ROWSIMAGE * params->config.parkEnter.spotUp)
            {
                // 达到阈值，记录当前编码器距离并准备好延迟
                delayTriggerReady = true;
                delayStartDist = sensor->Distance;
                printf("[Park] 1/4 spot visual trigger at y=%.2f, dist=%.2f, waiting for distance delay\n", 
                       forkCenterY, delayStartDist);
            }
        }
        // 如果已经准备好延迟，检查是否行驶了足够的距离
        if (delayTriggerReady)
        {
            float traveled = sensor->Distance - delayStartDist;
            if (traveled >= delayDistance)
            {
                // 延迟距离达到，切换进入 ENTER
                setStep(Step::ENTER);
                pointsEdgeLeftPast.clear();
                pointsEdgeRightPast.clear();
                delayTriggerReady = false; // 复位
                printf("[Park] 1/4 spot triggered by distance delay (traveled=%.2f)\n", traveled);
            }
        }
    }
    // 近处车位 2/3号（左后/右后）→ 第二个叉（保持原逻辑不变）
    else if (spots.spotEnable[1] || spots.spotEnable[2])
    {
        if (spots.forks.size() == 2)
        {
            if ((spots.forks[1].y + spots.forks[1].height / 2) > ROWSIMAGE * params->config.parkEnter.spotDown)
            {
                spots.times++;
                if (spots.times > 0)
                {
                    spots.times = 0;
                    setStep(Step::ENTER);
                    pointsEdgeLeftPast.clear();
                    pointsEdgeRightPast.clear();
                }
            }
        }
    }
}
        // 出库状态切换（硬编码，因出库参数已移除）
        if (countSes > 100)
        {
            if (timeout > 20) // 超时
                countRes++;   // 出库计数器
            if (countRes > 3)
                setStep(Step::FORKOUT); // 设置停车场新步骤
        }
        break;
    }

    case Step::ENTER: // 驶入停车位
    {
         cout <<"ENTER"<<endl;  // 注释调试输出
        // ----- 1. 进入状态初始化（仅首次执行） -----
        static bool initialized = false;
        static int16_t start_gyro = 0;      // 记录开始时的陀螺仪角度
        static int16_t start_distance = 0;  // 记录开始时的编码器计数值

        if (!initialized)
        {
            start_gyro = sensor->gyro_data;     // 保存当前航向角
            start_distance = sensor->Distance;  // 保存当前编码器值
            initialized = true;
        }

        // ----- 2. 判断目标车位方向 -----
        int16_t target_delta = 0;   // 目标旋转角度增量（度）
        if (spots.spotEnable[0] || spots.spotEnable[1])
        {
            target_delta = -params->config.parkEnter.enterAngle; // 使用配置的入库角度
        }
        else if (spots.spotEnable[2] || spots.spotEnable[3])
        {
            target_delta = params->config.parkEnter.enterAngle;  // 使用配置的入库角度
        }
        // ----- 3. 计算误差 -----
        int16_t target_gyro = start_gyro + target_delta;               // 目标航向角
        int16_t angle_error = target_gyro - sensor->gyro_data;         // 误差 = 目标 - 当前
        int16_t traveled = sensor->Distance - start_distance;          // 已行驶距离（编码器脉冲数）

        // 将角度误差直接赋给控制参数（供舵机闭环使用）
        params->ctrl.park_err = angle_error;

        // ----- 4. 判断入库完成 -----
        const int16_t DIST_THRESHOLD = params->config.parkEnter.enterDistThreshold; // 距离阈值

        if (traveled >= DIST_THRESHOLD)
        {
            setStep(Step::PARKING);   // 入库完成，进入停车状态
            initialized = false;      // 重置，供下次使用
        }
        break;
    }

    case Step::PARKING: // 停车
    {
        // cout <<"PARKING"<<endl;  // 注释调试输出
        params->ctrl.stop = true; // 停车标志
        timeout++;
        if (timeout > params->config.parkEnter.parkingDuration)        // 使用配置的停车时长
            setStep(Step::EXIT); // 出库
        break;
    }

    case Step::EXIT: // 驶出停车位（保持硬编码）
    {
         cout <<"EXIT"<<endl;
        static bool initialized = false;
        static int16_t start_distance = 0;   // 出库开始时的编码器值
        static uint8_t stage = 0;            // 0: 初始倒退, 1: 转向并继续倒退
        params->ctrl.stop = false; // 停车标志
        params->ctrl.back = true;
         cout <<"back  "<< params->ctrl.back<<endl;

        if (!initialized)
        {
            start_distance = sensor->Distance;
            stage = 0;                       // 从初始倒退阶段开始
            initialized = true;
        }

        int16_t traveled = sensor->Distance - start_distance; // 倒车编码器递减，注意方向

        const int16_t INIT_BACKUP_DIST = params->config.parkEnter.exitBackupDist;
        const int16_t DIST_THRESHOLD = params->config.parkEnter.exitDistThreshold;

        if (stage == 0)
        {
            params->ctrl.park_err = 0;          // 舵机中位，直行倒退
            if (traveled >= INIT_BACKUP_DIST)
            {
                 cout <<"backing  "<<endl;
                stage = 1;                      // 进入阶段1
                if (spots.spotEnable[0] || spots.spotEnable[1])
                {
                    exit_target_gyro = sensor->gyro_data - 85;     // 硬编码
                }
                else if (spots.spotEnable[2] || spots.spotEnable[3])
                {
                    exit_target_gyro = sensor->gyro_data + 85;      // 硬编码
                }
            }
        }
        else // stage == 1
        {
            int16_t angle_error = exit_target_gyro - sensor->gyro_data;
            params->ctrl.park_err = -angle_error;  // 传给舵机PID闭环
             cout<<"angerr"<<angle_error<<endl;
             cout<<"Diserr"<<traveled<<endl;

            if (traveled >= DIST_THRESHOLD)
            {
                setStep(Step::TRACKOUT);            // 出库完成，回到正常行驶状态
                initialized = false;              // 重置，供下次使用
            }
        }
        break;
    }

    case Step::TRACKOUT: // 出库巡线（保持硬编码）
    {   
        timeout++;
            bool gateNear = false;   // 每帧重置，只在本帧有效

            //[01] 道闸检测
            for (int i = 0; i < params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_GATE)
                {
                    int bottom = params->results[i].y + params->results[i].height;
                    printf("[TRACKOUT] Gate detected at y=%d, bottom=%d\n", params->results[i].y, bottom);

                    timeout = 0;
                    countRes = 0;

                    if (bottom > ROWSIMAGE * 0.40)  // 注意：阈值可调，可改成0.45等
                    {
                        gateNear = true;
                        stopping = true;
                        params->ctrl.stop = true;
                    }
                    // 即使 bottom 低于阈值，也重置超时，但不置停车
                }
            }

            // 如果本帧没有道闸，或道闸不在近处，清除停车
            if (!gateNear)
            {
                stopping = false;
                params->ctrl.stop = false;
            }

        //[02] 控制中心重规划
        spots.forks = findParkStation(params->results); // 搜索停车位坐标
        PredictResult resLeft;
        resLeft.score = 0;
        for (int i = 0; i < params->results.size(); i++) // 搜索左转箭头
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120 &&
                params->results[i].score > resLeft.score)
                resLeft = params->results[i];
        }

        if (spots.forks.size() > 0 || resLeft.score > 0)
        {
            PredictResult direction;
            if (resLeft.score > 0)
                direction = resLeft;
            else
                direction = spots.forks[0];
            PointX start = PointX(ROWSIMAGE - 20, COLSIMAGE / 2);
            PointX end = PointX(direction.y, direction.x + direction.width / 2);
            PointX mid = PointX((start.x + end.x) / 2, (start.y + end.y) / 2);
            vector<PointX> repairPoints = {start, mid, end};
            params->ctrl.centerEdge = Bezier(0.02, repairPoints);
            params->ctrl.fitting = true;
            if (spots.forks.size() > 0)
                timeout = 0;

            if (resLeft.score > 0 && (resLeft.y + resLeft.height / 2) > ROWSIMAGE * params->config.parkEnter.endForkRowRatio) // 硬编码
            {
                countRes++;
            }
        }

        // 出库状态切换（超时或箭头确认）
        // if (timeout > 65) // 硬编码
        //     countRes++; // 出库计数器
        if (countRes > 1) // 硬编码
        { 
            setStep(Step::FORKOUT); // 设置停车场新步骤
            params->ctrl.parkleft=true;
        }
        

        break;
    }

    case Step::FORKOUT: // 出库岔路转向（保持硬编码）
    {
        speedUp = 0;
        timeout++;
        // replanTracking(); // 车道线重绘（岔路左转）

        bool leftSign = false;
        for (int i = 0; i < params->results.size(); i++) // 搜索左转箭头
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120)
            {
                leftSign = true;
                break;
            }
        }
        if (leftSign) // 标志未丢失
        {
            countRes = 0;
        }
            
        else
            countRes++;

        if (timeout > 20 || countRes > 2) // 硬编码
        {   params->ctrl.parkleft=false;
            params->ctrl.countAcc = 50;        // 跳过缓加速，直接恢复速度
            params->ctrl.outlineCooldown = 90; // 出库后约4.5秒内禁用outlineCheck
            params->ctrl.yforkReset = true;    // 通知yfork复位，防止残留forkSeen误触发
            setStep(Step::NONE);               // 设置停车场新步骤
        }
        break;
    }
    }

    // 停车场内道闸停车：stopping标志生效时控制车辆停车，消失后恢复行驶
    if (stopping)
    {
        params->ctrl.stop = true;
    }
    else if (step == Step::TRACKIN || step == Step::TRACKOUT || step == Step::ENTER)
    {
        // 非PARKING步骤且gate已消失，清除停车标志恢复行驶
        params->ctrl.stop = false;
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmPark::show(Mat &img)
{
    if (params->mode != FsmMode::PARK)
        return;

    putText(img, "[5] Park", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    // 绘制边缘点
    for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeLeft[i].y, params->track->pointsEdgeLeft[i].x), 2,
               Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
    {
        circle(img, Point(params->track->pointsEdgeRight[i].y, params->track->pointsEdgeRight[i].x), 2,
               Scalar(0, 255, 255), -1); // 黄色点
    }

    // 绘制中心点集
    if (params->ctrl.fitting)
    {
        for (int i = 0; i < params->ctrl.centerEdge.size(); i++)
            circle(img, Point(params->ctrl.centerEdge[i].y, params->ctrl.centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
    }

    string str = "Enable";
    switch (step)
    {
    case Step::FORKIN:
        str = "Forkin";
        break;
    case Step::TRACKIN:
    {
        str = "Trackin";
        for (int i = 0; i < spots.forks.size(); i++) // 绘制停车位箭头标志
        {
            putText(img, to_string(i + 1), Point(spots.forks[i].x + spots.forks[i].width / 2, spots.forks[i].y + spots.forks[i].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
            cv::Rect rect(spots.forks[i].x, spots.forks[i].y, spots.forks[i].width, spots.forks[i].height);
            cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
        }

        // 透视变换视角
        Mat imgIpm = Mat::zeros(Size(COLSIMAGEIPM, ROWSIMAGEIPM), CV_8UC3); // 创建全黑图像
        for (int i = 0; i < spots.carsIpm.size(); i++)
            circle(imgIpm, Point(spots.carsIpm[i].x, spots.carsIpm[i].y), 3, Scalar(0, 0, 255), -1);

        for (int i = 0; i < spots.forksIpm.size(); i++)
            circle(imgIpm, Point(spots.forksIpm[i].x, spots.forksIpm[i].y), 3, Scalar(0, 255, 0), -1);

        // 绘制所有4个车位标签（实际布局：左前=1，左后=2，右后=3，右前=4）
        // 绿色=空闲(spotEnable=true)，红色=占用(spotEnable=false)
        if (spots.forks.size() > 0)
        {
            Scalar c1 = spots.spotEnable[0] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "1", Point(spots.forks[0].x + spots.forks[0].width / 2, spots.forks[0].y + spots.forks[0].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c1, 0.5);
        }
        if (spots.forks.size() > 1)
        {
            Scalar c2 = spots.spotEnable[1] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "2", Point(spots.forks[1].x + spots.forks[1].width / 2, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c2, 0.5);

            Scalar c3 = spots.spotEnable[2] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "3", Point(spots.forks[1].x + spots.forks[1].width / 2 + 100, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c3, 0.5);

            Scalar c4 = spots.spotEnable[3] ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            putText(img, "4", Point(COLSIMAGE - 31, spots.forks[1].y + spots.forks[1].height / 2),
                    cv::FONT_HERSHEY_TRIPLEX, 0.5, c4, 0.5);
        }
        savePicture("../res/samples/imgs/", imgIpm);
        savePicture("../res/samples/imgs/", img);
        break;
    }
    case Step::ENTER:
        str = "Enter";
        break;
    case Step::PARKING:
        str = "Parking";
        break;
    case Step::EXIT:
        str = "Exit";
        break;
    case Step::TRACKOUT:
        str = "TRACKOUT";
        break;
    case Step::FORKOUT:
        str = "Forkout";
        break;
    default:
        break;
    }
    putText(img, "PARK - " + str, Point(100, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 设置指定停车位
 */
void FsmPark::setParkSpotOverride(int spotNumber)
{
    std::cout << "[Park] setParkSpotOverride: " << spotNumber << std::endl;

    // 使能指定的停车位
    for (int i = 0; i < 4; i++)
    {
        spots.spotEnable[i] = false; // 先禁用所有停车位
    }

    if (spotNumber > 0 && spotNumber <= 4)
    {
        spots.spotEnable[spotNumber - 1] = true; // 使能指定的停车位
        std::cout << "[Park] Enabled spot " << spotNumber << std::endl;
    }
}

/**
 * @brief 设置指定停车位被占用
 */
void FsmPark::setParkSpotOccupied(int spotNumber)
{
    if (spotNumber > 0 && spotNumber <= 4)
    {
        spots.spotEnable[spotNumber - 1] = false; // 设置指定停车位被占用

        // 在计数器中增加，使其被判定为已占用
        spots.counter[spotNumber - 1] = 5; // 设置足够大的计数值

        // 为其他停车位设置不占用状态
        for (int i = 0; i < 4; i++)
        {
            if (i != (spotNumber - 1))
            {
                // 清除其他停车位的占用状态
                if (spots.counter[i] > 3)
                {
                    spots.spotEnable[i] = true;
                }
            }
        }
    }
}

/**
 * @brief 停车场数据复位
 *
 */
void FsmPark::reset()
{
    countSes = 0;      // AI场景识别计数器
    timeout = 0;       // 超时计数器
    step = Step::NONE; // 停车步骤
    waiting = false;   // 停车等待使能
    countWait = 0;     // 停车等待计数器
    countOut = 0;      // 出库检测计数
    countIn = 0;       // 入库矫正计数器
    speedUp = 0;       // 出库加速延迟计数器
    countFlow = 0;     // 直行计数器
    stopping = false;  // 确保复位时清除停车标志
}

/**
 * @brief 设置下阶段
 *
 * @param step
 */
void FsmPark::setStep(Step st)
{   
    countRes = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
    step = st;    // 停车步骤
    countIn = 0;  // 入库矫正计数器
    countFlow = 0;
    params->ctrl.back = false; // 倒车失能
    stopping = false;          // 切换状态时清除停车标志
}

/**
 * @brief 车道线重绘（岔路左转）
 *
 */
void FsmPark::replanTracking()
{
    params->track->pointsEdgeLeft.clear();  // 清空原来数据
    params->track->pointsEdgeRight.clear(); // 清空原来数据

    // 左车道线
    PointX startPoint = PointX(ROWSIMAGE - 10, 1);                                                // 入库补线起点:固定左下角
    PointX endPoint = PointX(ROWSIMAGE / 3, 1);                                                   // 入库补线终点
    PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
    vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
    vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
    params->track->pointsEdgeLeft = modifyEdge;

    // 右车道线
    startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.6);                                     // 入库补线起点:固定左下角
    endPoint = PointX(ROWSIMAGE / 3, 10);                                                     // 入库补线终点
    midPoint = PointX((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.35); // 入库补线中点（更靠下，左转更早）
    repairPoints = {startPoint, midPoint, endPoint};
    modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
    params->track->pointsEdgeRight = modifyEdge;
}
/**
 * @brief 停车入库车道线重绘
 *
 * @param left true：左转 | false：右转
 */
void FsmPark::replanTracking(bool left)
{
    params->track->pointsEdgeLeft.clear();  // 清空原来数据
    params->track->pointsEdgeRight.clear(); // 清空原来数据

    if (left)
    {
        // 左车道线
        PointX startPoint = PointX(ROWSIMAGE - 10, 1);                                                // 入库补线起点:固定左下角
        PointX endPoint = PointX(ROWSIMAGE / 3, 1);                                                   // 入库补线终点
        PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
        vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeLeft = modifyEdge;
        // 右车道线
        startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.8);                                  // 入库补线起点:固定左下角
        endPoint = PointX(ROWSIMAGE / 3, 1);                                                   // 入库补线终点
        midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        repairPoints = {startPoint, midPoint, endPoint};
        modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeRight = modifyEdge;
    }
    else
    {
        // 左车道线
        PointX startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.2);                                  // 入库补线起点:固定左下角
        PointX endPoint = PointX(ROWSIMAGE / 8, COLSIMAGE - 1);                                       // 入库补线终点
        PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
        vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeLeft = modifyEdge;
        // 右车道线
        startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE - 1);                                    // 入库补线起点:固定左下角
        endPoint = PointX(ROWSIMAGE / 8, COLSIMAGE - 1);                                       // 入库补线终点
        midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 入库补线中点
        repairPoints = {startPoint, midPoint, endPoint};
        modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeRight = modifyEdge;
    }
}

/**
 * @brief 搜索AI标志
 *
 * @param label
 * @return true
 * @return false
 */
bool FsmPark::findSymbols(vector<PredictResult> results, int label)
{
    for (int i = 0; i < results.size(); i++)
    {
        if (results[i].type == label) // AI标志
            return true;
    }

    return false;
}

/**
 * @brief 搜索AI标志（最顶上的一个）
 *
 * @param label
 * @param target 目标标志信息
 * @return true
 * @return false
 */
bool FsmPark::findSymbols(vector<PredictResult> results, int label, PredictResult &target)
{
    bool res = false;
    target.y = ROWSIMAGE - 1;
    for (int i = 0; i < results.size(); i++)
    {
        if (results[i].type == label) // AI标志
        {
            if (results[i].y < target.y && results[i].width < 120 && results[i].height < 120)
            {
                target = results[i];
                res = true; // 搜索到目标AI
            }
        }
    }

    return res;
}

/**
 * @brief 搜索停车位坐标
 *
 * @param results
 */
vector<PredictResult> FsmPark::findParkStation(vector<PredictResult> results)
{
    vector<PredictResult> resFork;
    //std::cout << "[Park] findParkStation: Searching in " << results.size() << " results" << std::endl;

    for (int i = 0; i < results.size(); i++)
    {
        if (results[i].type == LABEL_FORK)
        {
            // std::cout << "[Park] Found fork[" << i << "]: x=" << results[i].x
            //           << ", y=" << results[i].y
            //           << ", w=" << results[i].width
            //           << ", h=" << results[i].height
            //           << ", score=" << results[i].score << std::endl;

            if (results[i].width < 100 && results[i].height < 120 && results[i].y > 15)
            {
                resFork.push_back(results[i]);
            //    std::cout << "[Park] Fork[" << i << "] added to resFork" << std::endl;
            }
            else
            {
                // std::cout << "[Park] Fork[" << i << "] filtered out (size=" << results[i].width << "x" << results[i].height
                //           << ", y=" << results[i].y << ")" << std::endl;
            }
        }
    }

    // std::cout << "[Park] Total forks after initial filter: " << resFork.size() << std::endl;

    vector<PredictResult> resSpot;   // size=0:无停车位，size=1:停车位1/2，size=2:停车位1/2/3/4
    vector<PredictResult> resFilter; // 滤波后的坐标

    if (resFork.size() < 1) // 未检测AI标志
        return resSpot;

    if (resFork.size() > 0)
    {
        resFilter.push_back(resFork[0]);
        // std::cout << "[Park] Added first fork to filter" << std::endl;

        for (int i = 1; i < resFork.size(); i++) // Start from 1 since index 0 is already added
        {
            bool added = false;
            for (int n = 0; n < resFilter.size(); n++)
            {
                PointX centerFilter = getResultCenter(resFilter[n]);
                PointX centerResult = getResultCenter(resFork[i]);
                if (centerFilter.x > 0 && centerFilter.y > 0 &&
                    centerResult.x > 0 && centerResult.y > 0) // 数据有效
                {
                    if (abs(centerFilter.y - centerResult.y) > 30) // 两个坐标不重叠（降低阈值）
                    {
                        bool newFork = true;
                        for (int y = 0; y < resFilter.size(); y++) // 重复筛选
                        {
                            PointX centerF = getResultCenter(resFilter[y]);
                            if (abs(centerF.y - centerResult.y) < 30) // 数据重复
                            {
                                newFork = false;
                                break;
                            }
                        }
                        if (newFork)
                        {
                            resFilter.push_back(resFork[i]);
                            //std::cout << "[Park] Added fork[" << i << "] to filter (new fork)" << std::endl;
                            added = true;
                        }
                        else if (resFork[i].score > resFilter[n].score) // 更新数据
                        {
                            resFilter[n] = resFork[i];
                            //std::cout << "[Park] Updated fork[" << n << "] in filter (higher score)" << std::endl;
                            added = true;
                        }
                    }
                    else if (resFork[i].score > resFilter[n].score) // 更新数据
                    {
                        resFilter[n] = resFork[i];
                        //std::cout << "[Park] Updated fork[" << n << "] in filter (closer but higher score)" << std::endl;
                        added = true;
                    }
                }
            }
            if (!added)
            {
                //std::cout << "[Park] Fork[" << i << "] not added to filter (duplicate or invalid)" << std::endl;
            }
        }
    }

   // std::cout << "[Park] Final resFilter size: " << resFilter.size() << std::endl;

    if (resFilter.size() == 1) // 一个车位
    {
        resSpot = resFilter;
        //std::cout << "[Park] Returning 1 spot" << std::endl;
    }
    else if (resFilter.size() == 2) // 两个车位标识
    {
        // std::cout << "[Park] Found 2 spots!" << std::endl;
        // std::cout << "[Park] Spot 0: y=" << resFilter[0].y << ", center_y=" << (resFilter[0].y + resFilter[0].height / 2) << std::endl;
        // std::cout << "[Park] Spot 1: y=" << resFilter[1].y << ", center_y=" << (resFilter[1].y + resFilter[1].height / 2) << std::endl;

        if ((resFilter[0].y + resFilter[0].height / 2) < (resFilter[1].y + resFilter[1].height / 2))
        {
            resSpot = resFilter;
            //std::cout << "[Park] Spots in original order" << std::endl;
        }
        else // 车位号重新排序
        {
            resSpot.push_back(resFilter[1]);
            resSpot.push_back(resFilter[0]);
            //std::cout << "[Park] Spots reordered" << std::endl;
        }
    }
    else
    {
        //std::cout << "[Park] Unexpected resFilter size: " << resFilter.size() << std::endl;
    }

    //std::cout << "[Park] Returning " << resSpot.size() << " spots" << std::endl;
    return resSpot;
}

/**
 * @brief 获取AI检测目标的质心坐标
 *
 * @param res
 * @return PointX
 */
PointX FsmPark::getResultCenter(PredictResult res)
{
    PointX center;
    center.x = res.x + res.width / 2;
    center.y = res.y + res.height / 2;

    if (center.x < 0)
        center.x = 0;
    else if (center.x > COLSIMAGE)
        center.x = COLSIMAGE;

    if (center.y < 0)
        center.y = 0;
    else if (center.y > ROWSIMAGE)
        center.y = ROWSIMAGE;

    return center;
}

/**
 * @brief 检测停车位上是否有车
 *
 * @param results
 * @return vector<PredictResult>
 */
void FsmPark::findParkCars(vector<PredictResult> results)
{
    // 数据复位
    for (int i = 0; i < spots.carPark.size(); i++)
        spots.carPark[i].score = 0;

    if (spots.forks.size() != 2) // 基于岔路标志检测车位
        return;

    if (spots.forks[1].y < ROWSIMAGE / 4) // 图像清晰度限制
        return;

    spots.forksIpm.clear();
    spots.carsIpm.clear();
    Point2d spot1Ipm = ipm.homography(Point2d(spots.forks[0].y, spots.forks[0].x));
    Point2d spot2Ipm = ipm.homography(Point2d(spots.forks[1].y, spots.forks[1].x));
    spots.forksIpm.push_back(spot1Ipm);
    spots.forksIpm.push_back(spot2Ipm);
}

void FsmPark::resetLap()
{
    reset();
    spots.reset();
    pointsEdgeLeftPast.clear();
    pointsEdgeRightPast.clear();
    params->ctrl.stop = false;
    fitting = false;
    countRes = 0;
    stopping = false;   // 复位时清除停车标志
}