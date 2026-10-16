/**
 ********************************************************************************************************
 * @file obstacle.cpp
 * @brief 全局障碍物检测（锥桶/行人）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/obstacle.hpp"
#include "utils/tools.hpp"

FsmObstacle::FsmObstacle(std::shared_ptr<Params> par)
    : params(par)
{
}

FsmObstacle::~FsmObstacle()
{
}

void FsmObstacle::run(Mat &img)
{
    resultObs = PredictResult();
    params->ctrl.obsDetectFlag = false;
    // 不再重置 params->ctrl.slow，避免干扰 slow 模块
    // params->ctrl.slow = false;   // ← 删除

    if (params->track->pointsEdgeLeft.size() < ROWSIMAGE / 2 ||
        params->track->pointsEdgeRight.size() < ROWSIMAGE / 2)
        return;

    // 锥桶 + 行人检测
    vector<PredictResult> resultsObs;
    for (int i = 0; i < params->results.size(); i++)
    {
        if ((params->results[i].type == LABEL_CONE || params->results[i].type == LABEL_PERSON) &&
            (params->results[i].y + params->results[i].height) > ROWSIMAGE * 0.4 &&
            params->results[i].height < 100 && params->results[i].width < 90 &&
            params->results[i].height > 20 && params->results[i].width > 20)
            resultsObs.push_back(params->results[i]);
    }
    if (resultsObs.size() <= 0)
        return;

    params->ctrl.obsDetectFlag = true;
    params->ctrl.pianyi = 0.0;  // 先归零，防止残留上一帧偏移（未落入左右判定分支时）
    params->ctrl.slow = true;   // 有障碍物时强制减速

    // 选取距离最近的障碍物（面积最大）
    int areaMax = 0;
    int index = 0;
    for (int i = 0; i < resultsObs.size(); i++)
    {
        int area = resultsObs[i].width * resultsObs[i].height;
        if (area >= areaMax)
        {
            index = i;
            areaMax = area;
        }
    }
    resultObs = resultsObs[index];

    // 障碍物方向判定（左/右）
    int row = 0, width = COLSIMAGE;
    for (size_t i = 0; i < params->track->pointsEdgeLeft.size(); i++)
    {
        int w = abs(resultObs.y - params->track->pointsEdgeLeft[i].x);
        if (w < 2)
        {
            row = i;
            break;
        }
        if (w < width)
        {
            width = w;
            row = i;
        }
    }
    if (row > params->track->pointsEdgeRight.size() - 1)
        row = params->track->pointsEdgeRight.size() - 1;

    // 路径重规划：缩减车道线（双车道→单车道）+ 横向偏移量(pianyi)绕行
    // 废弃原贝塞尔曲线绕行实现
    int disLeft = resultsObs[index].x - params->track->pointsEdgeLeft[row].y;
    int disRight = params->track->pointsEdgeRight[row].y - (resultsObs[index].x + resultsObs[index].width);
    if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
        params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
        abs(disLeft) <= abs(disRight)) // 障碍物靠左
    {
        params->ctrl.pianyi = params->config.bizhangP1; // 横向偏移：向右绕行
        curtailTracking(false);                          // 车道线向右缩减，沿右半车道通过
    }
    else if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
             params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
             abs(disLeft) > abs(disRight)) // 障碍物靠右
    {
        params->ctrl.pianyi = -params->config.bizhangP2; // 横向偏移：向左绕行
        curtailTracking(true);                            // 车道线向左缩减，沿左半车道通过
    }
}

void FsmObstacle::resetLap()
{
    resultObs = PredictResult();
}

void FsmObstacle::show(Mat &img)
{
    if (resultObs.x > 0 && resultObs.y > 0)
    {
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
}

void FsmObstacle::curtailTracking(bool left)
{
    if (left) // 向左侧缩进
    {
        if (params->track->pointsEdgeRight.size() > params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeRight.resize(params->track->pointsEdgeLeft.size());

        for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        {
            params->track->pointsEdgeRight[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
    else // 向右侧缩进
    {
        if (params->track->pointsEdgeRight.size() < params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeLeft.resize(params->track->pointsEdgeRight.size());

        for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        {
            params->track->pointsEdgeLeft[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
}