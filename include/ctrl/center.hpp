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
 * @file center.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 车辆图像控制中心计算
 * @version 0.5
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include <memory>
#include <vector>
#include <string>
#include <climits>
#include <opencv2/opencv.hpp>
#include "utils/tools.hpp"
#include "utils/params.hpp"

using namespace cv;
using namespace std;

#define DIS_MOVE 48       // 偏移距离，对应赛道距离的一般，在我的打表软件中默认48像素对应20cm
#define MAX_POINT_NUM 240 // 无需修改
#define DIS_SECTION 75    // 使用centerMove时每一段点集的最大长度



/**
 * @brief 控制中心处理类
 */
class Center
{
public:
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
    double sigmaCenter = 0;      // 中心点集的方差

    /**
     * @brief 主拟合函数，根据当前策略计算控制中心
     * @param params 全局参数
     */
    void fitting(shared_ptr<Params> &params);

    /**
     * @brief 逆透视变换：将中心线点集从原始域变换到俯视图域
     * @param params 全局参数
     */
    void transformIPM(shared_ptr<Params> &params);

    /**
     * @brief 在图像上绘制边缘、中心点及控制信息
     * @param params 全局参数
     * @param img 绘制目标图像
     */
    void drawImage(shared_ptr<Params> &params, Mat &img);

    /**
     * @brief 设置循线策略（运行时可切换）
     * @param strategy 目标策略
     */
    inline void setStrategy(TrackStrategy strategy) { currentStrategy = strategy; }

    /**
     * @brief 获取当前策略
     */
    inline TrackStrategy getStrategy() const { return currentStrategy; }

    /**
     * @brief 从 IPM 中心线点集中获取指定行的目标点（最近邻）
     * @param targetRowIPM 目标行（IPM 坐标系中的 y 坐标）
     * @param params 全局参数
     * @return Point2d 目标点 (x=列, y=行)，如果中心线为空则返回 (-1,-1)
     */
    Point2d getTargetPointIPM(int targetRowIPM, shared_ptr<Params> &params);

private:
    string style = "";
    int countOut = 0;
    int timeout = 0;
    TrackStrategy currentStrategy = TrackStrategy::DUAL_EDGE; // 默认双边
    vector<PointX> lastCenterEdge; // 上一帧成功的中心线（用于惯性保持）

    // ----- 显示辅助 -----
    void showMode(Mat &img, FsmMode mode);

    // ----- 原有私有函数（保留） -----
    uint16_t searchBreakLeftDown(vector<PointX> pointsEdgeLeft);
    uint16_t searchBreakRightDown(vector<PointX> pointsEdgeRight);
    vector<PointX> centerCompute(vector<PointX> pointsEdge, int side);
    void validRowsCal(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight);
    void derailmentCheck(vector<PointX> pointsEdgeLeft, vector<PointX> pointsEdgeRight);

    // ----- 单边拟合函数（无弯道自适应，无过滤） -----
    bool fitLeftEdge(shared_ptr<Params> &params,
                     const vector<PointX>& edgePoints,
                     int nearOffset = 55,
                     int farOffset = 20);

    bool fitRightEdge(shared_ptr<Params> &params,
                      const vector<PointX>& edgePoints,
                      int nearOffset = 70,
                      int farOffset = 30);
};