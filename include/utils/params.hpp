#pragma once
/**
 ********************************************************************************************************
 * @file params.hpp
 * @brief 机器人状态参数（FSM共享传递）- 完整版
 * @version 1.3
 * @date 2025-04-08
 *
 * 包含：通用参数、停车场（含完整巡线参数）、Y型岔路、施工区、障碍物等
 * 新增：runP1、runP2 用于停车场 PD 控制
 ********************************************************************************************************
 */

#include <string>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include "utils/json.hpp"
#include "ctrl/track.hpp"

using namespace std;

enum class FsmMode
{
    NORMAL, FORK, PARK, BUSY, SLOW, FINE, STOP, CROSS, YFORK, OBSTACLE
};

enum class TrackStrategy {
    DUAL_EDGE,
    LEFT_EDGE,
    RIGHT_EDGE
};

struct Control
{
    cv::Point2d mubiao;
    cv::Point2d mubiaoni;
    double rad;
    double ld;
    bool stop = false;
    bool back = false;
    bool slow = false;
    bool obsDetectFlag = false;
    bool parkleft=false;
    bool crossStop = false;         // 斑马线停车标志（cross场景）
    double pianyi = 0.0;            // 避障偏移量（rad），未检测到障碍物时为0
    uint16_t servo = PWMSERVOMID;
    float speed = 0.0;
    int center = COLSIMAGE / 2;
    vector<PointX> centerEdge;
    vector<Point2d> centerEdgeIPM;
    Point2d anchorIPM = Point2d(-1, -1);
    double lyValue = 0.0;
    double kappa = 0.0;
    bool useStanley = true;
    int lineArea = 0;
    bool fitting = false;
    bool parking = false;
    int countAcc = 500;
    int countAccTotal = 0;          // 加速斜坡总帧数（精确控制）
    int outlineCooldown = 0;
    bool yforkReset = false;
    int park_err = 0;               // 停车场舵机闭环误差
};

struct Config
{
    // ==================== 通用速度参数 ====================
    float velHigh = 1.3;
    float velSlow = 0.5;
    float velPark = 6.0;
    float velBusy = 0.8;
    float velCross = 0.7;
    float velYfork = 0.7;

    float bizhangP1 = 0.2f;      // 避障横向偏移量(rad)：障碍物靠左时向右绕行
    float bizhangP2 = 0.3f;      // 避障横向偏移量(rad)：障碍物靠右时向左绕行

    float parkleft = -0.2f;

    // 停车场 PD 控制参数
    float runP1 = 2.2f;      // 比例系数
    float runP2 = 0.007f;    // 微分系数

    float turnP = 3.5;
    int park_looking_y=350;
    int looking_y = 270;
    int looking_offset = 90;
    int busy_looking_y = 270;      // 施工区drivingThrough阶段独立前视y坐标

    bool debug = false;
    bool uartEnable = true; // 串口通信使能（无下位机调试时设为false）
    bool saveImg = false;
    bool saveIpm = false;
    uint16_t rowCutUp = 10;
    uint16_t rowCutBottom = 20;
    float score = 0.2;
    int binary = -1;
    string model = "../res/models/yolov3_mobilenet_v1";
    int parkSpot = 0; // 目标停车位(1-4号车库)

    // ==================== 停车场参数（完整版） ====================
    struct ParkEnterParams {
        // ---- 进入识别 ----
        float enableForkRowRatio = 0.30f;
        float endForkRowRatio=0.30;

        // ---- 入库岔路 ----
        int   forkinTimeout = 20;

        // ---- 入库巡线（完整参数） ----
        float spotUp = 0.32f;
        float spotDown = 0.38f;
        float trackinGateStopRatio = 0.55f;
        float trackinGateExitRatio = 0.15f;
        int   trackinGateExitFrames = 5;
        int   trackinArrowExitFrames = 4;
        float trackinArrowRowRatio = 0.20f;
        int   trackinBezierInitFrames = 67;

        // ---- 入库控制 ----
        int   enterAngle = 75;
        int   enterDistThreshold = 40;

        // ---- 入库停车 ----
        int   parkingDuration = 20;

        // ---- 出库控制 ----
        int   exitBackupDist = 5;
        int   exitDistThreshold = 25;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(ParkEnterParams,
            enableForkRowRatio, endForkRowRatio,
            forkinTimeout,
            spotUp, spotDown,
            trackinGateStopRatio, trackinGateExitRatio, trackinGateExitFrames,
            trackinArrowExitFrames, trackinArrowRowRatio, trackinBezierInitFrames,
            enterAngle, enterDistThreshold,
            parkingDuration,
            exitBackupDist, exitDistThreshold)
    };

    ParkEnterParams    parkEnter;

    bool fork = true;
    bool park = true;
    bool spot = true;
    bool busy = true;
    bool slow = true;
    bool stop = true;
    bool cross = true;
    bool yfork = true;
    bool obstacle = true;

    // 序列化宏
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config,
        velHigh, velSlow, velPark, velBusy, velCross, velYfork,
        bizhangP1,bizhangP2,parkleft,
        runP1, runP2,
        turnP,
        park_looking_y,looking_y, looking_offset, busy_looking_y,
        debug, saveImg, saveIpm, rowCutUp, rowCutBottom,
        score, binary, model, parkSpot,
        fork, park, spot, busy, slow, stop, cross, yfork,
        parkEnter)
};

struct Params
{
public:
    Params()
    {
        string path = "../res/config.json";
        std::ifstream fileStr(path);
        if (!fileStr.good()) {
            std::cout << "Error: Params file path:[" << path << "] not find !!!" << std::endl;
            exit(-1);
        }
        nlohmann::json configs;
        fileStr >> configs;

        try {
            // 通用配置
            config.velHigh = configs["通用配置参数"]["velHigh"];
            config.velSlow = configs["通用配置参数"]["velSlow"];
            config.velPark = configs["通用配置参数"]["velPark"];
            config.velBusy = configs["通用配置参数"]["velBusy"];
            config.velCross = configs["通用配置参数"]["velCross"];
            config.velYfork = configs["通用配置参数"].value("velYfork", 0.7);

            config.bizhangP1 = configs["通用配置参数"].value("bizhangP1", 0.2f);
            config.bizhangP2 = configs["通用配置参数"].value("bizhangP2", 0.3f);
            config.parkleft = configs["通用配置参数"].value("parkleft", -0.2f);

            config.runP1 = configs["通用配置参数"].value("runP1", 2.2f);
            config.runP2 = configs["通用配置参数"].value("runP2", 0.007f);
            config.turnP = configs["通用配置参数"]["turnP"];
            config.park_looking_y = configs["通用配置参数"].value("park_looking_y", 350);
            config.looking_y = configs["通用配置参数"].value("looking_y", 270);
            config.looking_offset = configs["通用配置参数"].value("looking_offset", 90);
            config.busy_looking_y = configs["通用配置参数"].value("busy_looking_y", 270);
            config.debug = configs["通用配置参数"]["debug"];
            config.uartEnable = configs["通用配置参数"].value("uart", true);
            config.saveImg = configs["通用配置参数"]["saveImg"];
            config.saveIpm = configs["通用配置参数"]["saveIpm"];
            config.rowCutUp = configs["通用配置参数"]["rowCutUp"];
            config.rowCutBottom = configs["通用配置参数"]["rowCutBottom"];
            config.score = configs["通用配置参数"]["score"];
            config.binary = configs["通用配置参数"]["binary"];
            config.model = configs["通用配置参数"]["model"];

            // 停车场参数
            auto pjPark = configs.value("停车场参数", nlohmann::json::object());

            auto pjEnter0 = pjPark.value("进入识别", nlohmann::json::object());
            config.parkEnter.enableForkRowRatio = pjEnter0.value("enableForkRowRatio", 0.30f);
            config.parkEnter.endForkRowRatio = pjEnter0.value("endForkRowRatio", 0.30f);

            auto pjForkin = pjPark.value("入库岔路", nlohmann::json::object());
            config.parkEnter.forkinTimeout = pjForkin.value("forkinTimeout", 20);

            auto pjTrackin = pjPark.value("入库巡线", nlohmann::json::object());
            config.parkEnter.spotUp = pjTrackin.value("spotUp", 0.32f);
            config.parkEnter.spotDown = pjTrackin.value("spotDown", 0.38f);
            config.parkEnter.trackinGateStopRatio = pjTrackin.value("trackinGateStopRatio", 0.55f);
            config.parkEnter.trackinGateExitRatio = pjTrackin.value("trackinGateExitRatio", 0.15f);
            config.parkEnter.trackinGateExitFrames = pjTrackin.value("trackinGateExitFrames", 5);
            config.parkEnter.trackinArrowExitFrames = pjTrackin.value("trackinArrowExitFrames", 4);
            config.parkEnter.trackinArrowRowRatio = pjTrackin.value("trackinArrowRowRatio", 0.20f);
            config.parkEnter.trackinBezierInitFrames = pjTrackin.value("trackinBezierInitFrames", 67);

            auto pjEnterStep = pjPark.value("入库控制", nlohmann::json::object());
            config.parkEnter.enterAngle = pjEnterStep.value("enterAngle", 75);
            config.parkEnter.enterDistThreshold = pjEnterStep.value("enterDistThreshold", 40);

            auto pjParking = pjPark.value("入库停车", nlohmann::json::object());
            config.parkEnter.parkingDuration = pjParking.value("parkingDuration", 20);

            auto pjExitStep = pjPark.value("出库控制", nlohmann::json::object());
            config.parkEnter.exitBackupDist = pjExitStep.value("exitBackupDist", 5);
            config.parkEnter.exitDistThreshold = pjExitStep.value("exitDistThreshold", 25);

            // 功能使能（顶层平铺，参考版结构）
            config.fork = configs.value("fork", true);
            config.park = configs.value("park", true);
            config.spot = configs.value("spot", true);
            config.busy = configs.value("busy", true);
            config.slow = configs.value("slow", true);
            config.stop = configs.value("stop", true);
            config.cross = configs.value("cross", true);
            config.yfork = configs.value("yfork", true);
            config.obstacle = configs.value("obstacle", true);
            config.parkSpot = configs.value("parkSpot", 0);
        }
        catch (const nlohmann::detail::exception &e) {
            std::cerr << "Json Params Parse failed :" << e.what() << std::endl;
            exit(-1);
        }

        mode = FsmMode::NORMAL;
        modeLast = FsmMode::NORMAL;
        track = make_shared<Track>();
        track->rowCutUp = config.rowCutUp;
        track->rowCutBottom = config.rowCutBottom;
    };
    ~Params() {};

    Control ctrl;
    Config config;
    FsmMode mode, modeLast;
    shared_ptr<Track> track;
    std::vector<PredictResult> results;
    // 道闸检测缓存（解决AI推理线程与主控制循环异步时序导致的检测丢失）
    PredictResult gateCache;     // 最近一次道闸检测结果
    int gateCacheAge = 999;      // 距上次道闸检测的帧数（0=本帧检测到，越大越久）
    bool quit = false;           // 斑马线停车完成后的退出标志（主循环统一执行舵机归中后退出）
    int alertDecelCount = 0;

private:
};