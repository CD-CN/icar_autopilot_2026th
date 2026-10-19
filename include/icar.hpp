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
 * @file icar.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"
#include "fsm/fork.hpp"
#include "fsm/park.hpp"
#include "fsm/stop.hpp"
#include "fsm/cross.hpp"
#include "fsm/yfork.hpp"
#include "fsm/slow.hpp"
#include "fsm/obstacle.hpp"
#include "com/uart.hpp"
#include "utils/detection.hpp"
#include "utils/show.hpp"
#include "utils/loop.hpp"
#include "ctrl/predeal.hpp"
#include "ctrl/center.hpp"
#include "ctrl/motion.hpp"
using namespace std;
using namespace cv;

class Icar
{
private:
    /**
     * @brief 状态机管理
     *
     */
    struct FsmFactory
    {
        shared_ptr<FsmBusy> busy;               // 避障控制
        shared_ptr<FsmPark> park;               // 停车场控制
        shared_ptr<FsmStop> stop;               // 停车区控制
        shared_ptr<FsmCross> cross;             // 斑马线停车控制
        shared_ptr<FsmFork> fork;               // 停车场岔路控制
        shared_ptr<FsmSlow> slow;               // 慢行区控制
        shared_ptr<FsmObstacle> obstacle;       // 全局障碍物检测（锥桶/行人）
        shared_ptr<FsmYfork> yfork;             // Y型岔路口控制
    };

    FsmFactory fsmFactory;                // 状态机管理
    shared_ptr<Predeal> predeal;          // 图像预处理类
    shared_ptr<Show> show;                // 初始化UI显示窗口
    shared_ptr<cv::VideoCapture> capture; // Opencv相机类
    shared_ptr<Detection> detection;      // 目标检测类
    shared_ptr<Uart>uart;            // TCP客户端通信类
    shared_ptr<Params> params;            // 车辆状态参数（FSM共享传递）
    shared_ptr<Loops> loops;              // 子线程循环
    shared_ptr<Center> center;            // 控制中心处理类
    shared_ptr<Motion> motion;            // 运动控制器

    // 全局共享数据链
    cv::Mat imgShare;
    cv::Point2d lastRightTargetIPM = cv::Point2d(-1, -1); // 右边线寻迹：上一次有效目标点（丢线时惯性沿用，不回退中心线）
    std::mutex mtxImg;
    std::condition_variable cvImg;
    std::atomic<bool> readyImg{false};
    std::mutex mtxRes;
    std::atomic<bool> readyRes{false};

    // 相机采集线程：与主控制循环解耦，避免 capture->read 的 IO 阻塞
    cv::Mat imgCapture;                       // 最新帧缓冲（覆盖式更新）
    std::mutex mtxCapture;
    std::condition_variable cvCapture;
    std::atomic<bool> readyCapture{false};    // 是否有未消费的新帧
    std::atomic<bool> captureRunning{false};  // 采集线程运行标志
    std::thread captureThread;

    /**
     * @brief 相机采集线程：持续读取最新帧到共享缓冲
     *        覆盖式更新（丢旧保新），保证主循环总是拿到最新画面
     */
    void runCapture()
    {
        while (captureRunning.load())
        {
            cv::Mat img;
            if (!capture->read(img) || img.empty())
            {
                // 读取失败时短暂退避，避免 CPU 空转
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(mtxCapture);
                imgCapture = img; // 浅拷贝即可，VideoCapture 每帧返回独立缓冲
                readyCapture = true;
            }
            cvCapture.notify_one();
        }
    }

    /**
     * @brief AI 模型推理
     *
     */
    void runModel()
    {
        std::unique_lock<std::mutex> lock(mtxImg);
        cvImg.wait(lock, [this]
                   { return readyImg.load(); });
        cv::Mat img = imgShare.clone(); // 图像拷贝出来再释放锁
        readyImg = false;
        lock.unlock();

        // 启动AI推理
        detection->inference(img);
        std::lock_guard<std::mutex> lock_result(mtxRes);
        params->results = detection->results;
        // 更新道闸检测缓存：AI线程每次推理后检查是否检测到道闸，
        // 若检测到则缓存结果并重置帧龄；否则帧龄递增。
        // 解决AI推理与主控制循环异步时序导致park.cpp读不到道闸的问题。
        bool gateFound = false;
        for (auto &r : detection->results)
        {
            if (r.type == LABEL_GATE)
            {
                params->gateCache = r;
                params->gateCacheAge = 0;
                gateFound = true;
                break;
            }
        }
        if (!gateFound)
            params->gateCacheAge++;
        readyRes = true;
    }

    /**
     * @brief 有限状态机任务执行
     *
     */
    void runFsm(Mat &img)
    {
        if (params->mode == FsmMode::FORK ||
            params->mode == FsmMode::BUSY ||
            params->mode == FsmMode::SLOW ||
            params->mode == FsmMode::YFORK) // 状态复位
            params->mode = FsmMode::NORMAL;

        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        params->mode = fsmFactory.cross->getMode();

        // ===== 路线隔离：仅在当前圈使能时调用对应FSM =====
        if (params->config.park)
        {
            if (params->mode == FsmMode::NORMAL || params->mode == FsmMode::PARK || params->mode == FsmMode::CROSS)
            {
                fsmFactory.park->run(img);
                FsmMode mode = fsmFactory.park->getMode();
                if (mode != FsmMode::NORMAL)
                    params->mode = mode;
            }
        }
        if (params->config.fork)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.fork->run(img);
                params->mode = fsmFactory.fork->getMode();
            }
        }
        if (params->config.yfork)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.yfork->run(img);
                params->mode = fsmFactory.yfork->getMode();
            }
        }
        if (params->config.slow)
        {
            fsmFactory.slow->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.slow->getMode();
        }
        if (params->config.busy)
        {
            fsmFactory.busy->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.busy->getMode();
        }

        // 全局障碍物检测（锥桶/行人）
        // 施工区：等箭头(ENABLE)/区内巡线(RUNNING)阶段启用避障；
        //         进区/出区转向(FORKIN/FORKOUT)阶段边线为贝塞尔重绘，不检测。
        // 停车场/Y型岔路口期间不检测（保持原有逻辑）。
        // 避障模块未运行的帧需清除检测标志，防止残留的 pianyi 偏移影响后续转向（如YFORK）。
        bool obsRun =
            params->config.obstacle &&
            params->mode != FsmMode::PARK &&
            params->mode != FsmMode::YFORK &&
            !(params->mode == FsmMode::BUSY && !fsmFactory.busy->inAvoidPhase());
        if (obsRun)
            fsmFactory.obstacle->run(img);
        else
            params->ctrl.obsDetectFlag = false;

        // 报警目标减速检测（非蜂鸣器，仅控制减速）
        {
            bool decelFound = false;
            {
                std::lock_guard<std::mutex> lock(mtxRes);
                for (auto &r : params->results)
                {
                    bool posOk = false;
                    if (r.type == LABEL_LIMIT || r.type == LABEL_PARK)
                        posOk = (r.x > COLSIMAGE * 0.8 || r.x + r.width < COLSIMAGE * 0.2); // 左右两侧1/5
                    else if (r.type == LABEL_CONE || r.type == LABEL_PERSON || r.type == LABEL_UNLIMIT)
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2; // 底部4/5区域
                    else if (r.type == LABEL_BUSY)
                    {
                        if (params->config.busy)
                            continue;
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2;
                    }
                    else
                        continue;

                    if (posOk)
                    {
                        decelFound = true;
                        break;
                    }
                }
            }
            if (decelFound)
                params->alertDecelCount = 7;
            else if (params->alertDecelCount > 0)
                params->alertDecelCount--;
        }

        if (params->mode != params->modeLast)
        {
            params->modeLast = params->mode;
        }
    }

public:
    /**
     * @brief 参数初始化
     *
     */
    Icar()
    {
        params = make_shared<Params>();                        // 初始化参数
        center = make_shared<Center>();                        // 控制中心处理类
        predeal = make_shared<Predeal>(params->config.binary); // 图像预处理类
        detection = make_shared<Detection>(params->config.model,
                                           params->config.score); // AI模型初始化

        // 初始化串口
        uart = make_shared<Uart>();
        if (uart->open("/dev/ttyUSB0") < 0)
        {
            printf("[Error]: Uart init failed!!!\n");
            exit(-1);
        }
        uart->startReceive();

        // 相机初始化
        // USB摄像头初始化
        // 显式指定后端，避免OpenCV默认走GStreamer导致 "Embedded video playback halted /
        // unable to start pipeline / pipeline have not been created" 警告
        // debug/实车均从摄像头读取实时画面
        capture = make_shared<cv::VideoCapture>("/dev/video0", cv::CAP_V4L2); // 摄像头用V4L2
        if (!capture->isOpened())
        {
            printf("[Error]: Can not open video device!!!\n");
            exit(-1);
        }
        capture->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // MJPG格式，多数USB摄像头原生支持
        capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像分辨率
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像分辨率
        capture->set(cv::CAP_PROP_FPS, 30);                  // 设置帧率：与控制周期 30FPS 对齐

        if (params->config.debug)
        {
            show = make_shared<Show>(4); // 调试UI初始化（Bin/Track/Ctrl/IPM四窗口）
        }

        // FSM有限状态机初始化
        fsmFactory.busy = make_shared<FsmBusy>(params);         // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params, uart);   // 停车场控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);         // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params);       // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);         // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);         // 慢行区控制实例化
        fsmFactory.obstacle = make_shared<FsmObstacle>(params); // 全局障碍物检测实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params);       // Y型岔路口控制实例化

        motion = make_shared<Motion>();                          // 运动控制器


        // 启动AI推理子线程（与控制周期统一为 30FPS）
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Icar::runModel, this));
        loops->start(); // RL开始推理

        // 启动相机采集子线程（与主控制循环解耦，避免 capture->read 阻塞控制周期）
        captureRunning = true;
        captureThread = std::thread(&Icar::runCapture, this);

        printf("[OK]: Params initial succeed!!!\n");
    };
    ~Icar()
    {
        captureRunning = false;
        cvCapture.notify_all();
        if (captureThread.joinable())
            captureThread.join();
    };

    /**
     * @brief 程序主循环
     *
     */
    void running()
    {
        //[01] 视频源读取（由相机采集线程提供最新帧，避免 IO 阻塞主控制周期）
        cv::Mat img;
        {
            std::unique_lock<std::mutex> lock(mtxCapture);
            if (!readyCapture.load())
            {
                // 首帧未就绪时最多等待一个控制周期（≈33ms），仍无则跳过本次
                cvCapture.wait_for(lock, std::chrono::milliseconds(33),
                                    [this] { return readyCapture.load(); });
            }
            if (!readyCapture.load())
                return;
            img = imgCapture;       // 浅拷贝 Mat header
            imgCapture = cv::Mat();  // 清空共享缓冲，避免重复消费同一帧
            readyCapture = false;
        }

        //[02] 图像存储
        if (params->config.saveImg && !params->config.debug) // 存储原始图像
            savePicture(img);
        else if (params->config.saveImg && params->config.debug) // 存储调式图像
            show->save = true;

        //[03] 图像预处理
        cv::Mat imgBin;
        cv::Mat imgBinni;
        /*---------------子线程共享数据，避免浅拷贝-----------------*/
        // 收窄锁范围：仅在共享图像给AI线程时持锁，避免长时间阻塞AI推理线程
        {
            std::lock_guard<std::mutex> lock(mtxImg);
            imgShare = img.clone();
            readyImg = true;
            cvImg.notify_one();
        }
        /*-------------------------------------------------------*/
        imgBin = predeal->binaryzation(img); // 图像二值化
        //[04] 赛道识别（人工接管期间也运行双边线循迹，保证接管结束后中心线就绪）
        params->track->handle(imgBin);
        if (params->config.debug)
        {
            if (!imgBin.empty())
                show->setNewWindow(1, "Bin", imgBin);
            cv::Mat imgTrack = img.clone();
            params->track->drawImage(imgTrack); // 图像绘制赛道识别结果
            show->setNewWindow(2, "Track", imgTrack);
            cv::Mat imgIpm;
            ipm.homography(img, imgIpm); // 逆透视变换
            // 绘制Pure Pursuit预瞄锚点（红色圆点）和车辆位置（蓝色圆点）
            if (params->ctrl.centerEdgeIPM.size() >= 2)
            {
                Point2d &anc = params->ctrl.anchorIPM;
                if (anc.x >= 0 && anc.y >= 0)
                {
                    circle(imgIpm, Point((int)anc.x, (int)anc.y), 6, Scalar(0, 0, 255), -1); // 红色锚点
                    line(imgIpm, Point(COLSIMAGEIPM / 2, 499), Point((int)anc.x, (int)anc.y), Scalar(0, 0, 255), 1); // 连线
                }
                circle(imgIpm, Point(COLSIMAGEIPM / 2, 499), 4, Scalar(255, 0, 0), -1); // 蓝色车辆位置
            }
            show->setNewWindow(4, "IPM", imgIpm);
            if (params->config.saveIpm && params->config.saveImg)
                savePicture(imgIpm); // 保存图像
        }

        //[05] 有限状态机任务执行
        params->ctrl.fitting = false;
        runFsm(imgBin);

        //[06] 控制中心计算
        {
            // ===== 1. 设置策略 =====
            if (params->mode == FsmMode::PARK || params->mode == FsmMode::BUSY)
            {
                // 停车场/施工区模式：使用双边巡线（中心线），以便利用贝塞尔补线
                center->setStrategy(TrackStrategy::DUAL_EDGE);
            }
            // 其他模式（NORMAL/YFORK 等）不设置策略，保持 Center 默认（DUAL_EDGE）
            // YFORK 单边转向由 yfork 删线驱动，Center 内部已兼容单边线生成中心线

            // ===== 2. 生成中心线 =====
            center->fitting(params);
            center->transformIPM(params);

            // ===== 3. 选择目标行（前视距离） =====
            int targetRow;
            if (params->mode == FsmMode::PARK) {
                targetRow = params->config.park_looking_y;   // 停车场使用较小的前视距离
            }
            else if(params->mode == FsmMode::BUSY)
            {targetRow = params->config.busy_looking_y; }
             else {
                targetRow = params->config.looking_y;        // 其他模式使用默认前视距离
            }

            // ===== 4. 选择目标点来源 =====
            Point2d targetIPM;
            bool targetFound = false;

            if (params->mode == FsmMode::PARK || params->mode == FsmMode::BUSY || params->mode == FsmMode::YFORK) {
                // 停车场/施工区/Y型岔路模式：从中心线提取目标点
                // 施工区退出左转时 centerEdge 已被 Bezier 覆盖（fitting=true），此处自动使用 Bezier 路径
                Point2d targetIPM_center = center->getTargetPointIPM(targetRow, params);
                if (targetIPM_center.x >= 0 && targetIPM_center.y >= 0) {
                    targetIPM = targetIPM_center;
                    targetFound = true;
                }
                // 特殊区域不走右边线，清除缓存，避免出区后沿用过期目标点
                lastRightTargetIPM = Point2d(-1, -1);
            }
            else {
                // 右边线单边寻迹：右边缘 + 偏移量；丢线时不回退中心线，沿用上一次有效目标点
                const auto &edgeRight = params->track->pointsEdgeRight;
                if (edgeRight.size() >= 5) {
                    vector<Point2d> edgeRightIPM;
                    for (const auto &p : edgeRight) {
                        Point2d srcPt(p.y, p.x);
                        Point2d ipmPt = ipm.homography(srcPt);
                        if (ipmPt.x >= 0 && ipmPt.x < COLSIMAGEIPM && ipmPt.y >= 0 && ipmPt.y < ROWSIMAGEIPM)
                            edgeRightIPM.push_back(ipmPt);
                    }
                    int minDist = INT_MAX;
                    for (const auto &pt : edgeRightIPM) {
                        int dist = abs((int)pt.y - targetRow);
                        if (dist < minDist) {
                            minDist = dist;
                            targetIPM = pt;
                            targetFound = true;
                            if (dist == 0) break;
                        }
                    }
                    if (targetFound) {
                        int offset = params->config.looking_offset;
                        targetIPM.x = max(0.0, targetIPM.x - offset);
                        targetIPM.y = min((double)ROWSIMAGEIPM - 1, targetIPM.y);
                    }
                }
                if (targetFound) {
                    lastRightTargetIPM = targetIPM; // 记录本次有效目标点
                }
                else if (lastRightTargetIPM.x >= 0 && lastRightTargetIPM.y >= 0) {
                    // 右边线丢失（十字路口/缺口）：保持上一次目标点，延续当前转弯趋势
                    targetIPM = lastRightTargetIPM;
                    targetFound = true;
                }
            }

            // ===== 6. 赋值 =====
            if (targetFound) {
                params->ctrl.anchorIPM = targetIPM;
                params->ctrl.mubiaoni = Point((int)targetIPM.x, (int)targetIPM.y);
            } else {
                params->ctrl.anchorIPM = Point2d(-1, -1);
                params->ctrl.mubiaoni = Point(-1, -1);
            }
        }
        //[07] 车辆运动控制
        motion->poseControl(params, fsmFactory.park, fsmFactory.busy);
        motion->speedControl(params);

        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(img);        // 图像绘制AI结果
            center->drawImage(params, img); // 图像绘制控制路径
            show->setNewWindow(3, "Ctrl", img);

            show->show(); // 渲染并显示所有调试窗口
            uart->carControl(0, params->ctrl.servo, 0);
        }
        else // 实车控制
        {
            // 显示控制图像
            detection->drawBox(img);        // 图像绘制AI结果
            center->drawImage(params, img); // 图像绘制控制路径
            std::string str1 = "Dis: " + doble2String(uart->Distance.load(), 2);
            putText(img, str1, Point(COLSIMAGE - 100, 140), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); //距离
            std::string str2 = "Ang: " + doble2String(uart->gyro_data.load(), 2);
            putText(img, str2, Point(COLSIMAGE - 100, 160), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 陀螺仪角度
            //show->setNewWindow(1, "Ctrl", img);

             // 显示逆透视图像
            //  cv::Mat imgIpm;
            //  ipm.homography(img, imgIpm); // 逆透视变换
            //  show->setNewWindow(2, "IPM", imgIpm);

            // show->show();

            // 无论手动/自动模式，每帧发送控制指令（MCU需要持续PWM更新）
            // flag字节保留以兼容下位机帧格式，蜂鸣器响应逻辑已移除，恒为0
            uart->carControl(params->ctrl.speed, params->ctrl.servo, 0);
           // cout<<"uart"<<uart->gyro_data<<"dis"<<uart->Distance<<endl;
        }

        if (params->quit) // 斑马线停车触发退出：先舵机归中再退出进程
        {
            params->ctrl.servo = PWMSERVOMID; // 舵机归中
            params->ctrl.speed = 0;           // 停车
            uart->carControl(0, PWMSERVOMID, 0); // 发送归中+停车指令
            printf("-----> System Exit (servo centered)! <-----\n");
            exit(0); // 程序退出
        }
    }
};
