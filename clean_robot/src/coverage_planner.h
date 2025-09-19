/**
 * @author lijun
 * @date 2025-1-1
 * @brief Coverage Path Planning Using BCD Algorithm
 * 源代码是借鉴 Richey Huang  进行了少量了修改整理！
 * 站在前人的肩膀上可以走的更快！！！
 */
#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace coverageplanner
{

    const int TOP_LEFT = 0;     // 左上角
    const int BOTTOM_LEFT = 1;  // 左下角
    const int BOTTOM_RIGHT = 2; // 右下角
    const int TOP_RIGHT = 3;    // 右上角

    // 用于存储边界（外边界、障碍物边界）点集属性
    enum EventType
    {
        IN,
        IN_TOP,
        IN_BOTTOM,
        OUT,
        OUT_TOP,
        OUT_BOTTOM,

        INNER_IN,
        INNER_IN_TOP,
        INNER_IN_BOTTOM,
        INNER_OUT,
        INNER_OUT_TOP,
        INNER_OUT_BOTTOM,

        IN_EX,
        IN_TOP_EX,
        IN_BOTTOM_EX,
        OUT_EX,
        OUT_TOP_EX,
        OUT_BOTTOM_EX,
        INNER_IN_EX,
        INNER_IN_TOP_EX,
        INNER_IN_BOTTOM_EX,
        INNER_OUT_EX,
        INNER_OUT_TOP_EX,
        INNER_OUT_BOTTOM_EX,

        MIDDLE,
        CEILING,
        FLOOR,
        UNALLOCATED // 未分配的(主要是用于初始化)
    };

    // 定义二维点集
    class Point2D
    {
    public:
        Point2D()
        {
            x = INT_MAX;
            y = INT_MAX;
        }
        Point2D(int x_pos, int y_pos)
        {
            x = x_pos;
            y = y_pos;
        }
        Point2D(const Point2D &point)
        {
            x = point.x;
            y = point.y;
        }
        int x;
        int y;

        ////////////////////////////
        bool operator<(const Point2D &p2) const
        {
            return (this->x < p2.x || (this->x == p2.x && this->y < p2.y));
        }

        bool operator==(const Point2D &p2) const
        {
            return (this->x == p2.x && this->y == p2.y);
        }

        bool operator!=(const Point2D &p2) const
        {
            return !(*this == p2);
        }
    };

    /** 多边形顶点按照逆时针旋转排序 **/
    typedef std::vector<Point2D> Polygon;
    typedef std::vector<Polygon> PolygonList;
    typedef std::deque<Point2D> Edge;

    // 存放所有轮廓点集
    class Event
    {
    public:
        Event(int obstacle_idx, int x_pos, int y_pos, EventType type = UNALLOCATED)
        {
            obstacle_index = obstacle_idx;
            x = x_pos;
            y = y_pos;
            event_type = type;
            original_index_in_slice = INT_MAX;
            isUsed = false;
        }

        int x;
        int y;
        int original_index_in_slice; // 原始索引
        int obstacle_index;
        EventType event_type;
        bool isUsed; // 是否使用过

        bool operator<(const Event &e2)
        {
            return (this->x < e2.x || (this->x == e2.x && this->y < e2.y) || (this->x == e2.x && this->y == e2.y && this->obstacle_index < e2.obstacle_index));
        }
    };
    // 也可以用仿函数实现类中运算符重载的功能
    // bool operator<(const Event &e1, const Event &e2)
    // {
    //     return (e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y) || (e1.x == e2.x && e1.y == e2.y && e1.obstacle_index < e2.obstacle_index));
    // }

    class CellNode
    {
    public:
        CellNode()
        {
            isVisited = false;
            isCleaned = false;
            parentIndex = INT_MAX;
            cellIndex = INT_MAX;
        }
        bool isVisited;
        bool isCleaned;
        Edge ceiling; // 天花板点集
        Edge floor;   // 地平面点集
        int parentIndex;
        std::deque<int> neighbor_indices;
        int cellIndex;
    };

    /**
     * @brief 覆盖规划算法的核心
     *
     */
    class CoveragePlanner
    {
    public:
        /**
         * @brief clean_map 需要清洁覆盖的地图
         * @param robot_radius 机器人半径（默认是圆形）
         * @param clean_distance 清扫间距（覆盖弓字形半径）
         * @return std::deque<Point2D> 路径点集
         */
        std::deque<Point2D> planner(const cv::Mat &clean_map, double robot_radius, double clean_distance , Point2D start);

        ///////////////////////////////////////////////////////////////////////////////
    private:
        /**
         * @brief 提取障碍物轮廓 和 外边界轮廓
         * @param original_map 原始地图
         * @param wall_contours 沿边轮廓（外）
         * @param obstacle_contours 障碍物轮廓（内）
         * @param robot_radius 机器人半径（用于膨胀地图）
         * @return void
         */
        void extractContours(const cv::Mat &original_map, std::vector<std::vector<cv::Point>> &wall_contours, std::vector<std::vector<cv::Point>> &obstacle_contours, double robot_radius);

        /**
         * @brief 显示提取到的轮廓
         * @param map 原始底图
         * @param contours 需要显示的轮廓
         * @return void
         */
        void showExtractedContours(const cv::Mat &map, const std::vector<std::vector<cv::Point>> &contours);

        /**
         * @brief 将障碍物点集密集化
         * @param original_map 原始地图
         * @param obstacle_contours 障碍物轮廓
         * @return PolygonList 密集点轮廓
         */
        PolygonList constructObstacles(const cv::Mat &original_map, const std::vector<std::vector<cv::Point>> &obstacle_contours);

        /**
         * @brief 将外轮廓点集密集化
         * @param original_map 原始地图
         * @param wall_contour 墙体轮廓
         * @return Polygon 密集墙体轮廓
         */
        Polygon constructWall(const cv::Mat &original_map, std::vector<cv::Point> &wall_contour);

        // 得到区域分解轮廓
        /**
         * @brief 得到区域分解轮廓
         * @param wall_contours 墙体轮廓
         * @param obstacle_contours 障碍物轮廓
         * @param wall 密集墙体
         * @param obstacles 密集障碍物
         * @return std::vector<CellNode> 区域轮廓集合
         */
        std::vector<CellNode> constructCellGraph(const cv::Mat &original_map, const std::vector<std::vector<cv::Point>> &wall_contours, const std::vector<std::vector<cv::Point>> &obstacle_contours, const Polygon &wall, const PolygonList &obstacles);

        /**
         * @brief 直行区域分解的具体函数
         * @param cell_graph 引用返回计算结果
         * @param slice_list 以相同x为一组的特征点
         * @return void
         */
        void executeCellDecomposition(std::vector<CellNode> &cell_graph, const std::deque<std::deque<Event>> &slice_list);

        // 一个简单的条件计数器
        int countCells(const std::deque<Event> &slice, int curr_idx);
        //// ============== bcd 分解过程 ==============
        void executeOpenOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite = false);

        void executeOpenOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite = false);

        void executeFloorOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, const Point2D &floor_point);

        void executeCeilOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, const Point2D &ceil_point);

        void executeInnerCloseOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D inner_out);

        void executeInnerCloseOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom);

        void executeInnerOpenOperation(std::vector<CellNode> &cell_graph, Point2D inner_in);

        void executeInnerOpenOperation(std::vector<CellNode> &cell_graph, Point2D inner_in_top, Point2D inner_in_bottom);

        void executeCloseOperation(std::vector<CellNode> &cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D c, Point2D f, bool rewrite = false);

        /**
         * @brief 生产障碍物轮廓点集的事件类型
         * @param map 原始地图
         * @param polygons 障碍物轮廓点集
         * @return std::vector<Event> 障碍物关键点的属性
         */
        std::vector<Event> generateObstacleEventList(const cv::Mat &map, const PolygonList &polygons);

        /**
         * @brief 生产墙体点集的事件类型
         * @param map 原始地图
         * @param external_contour 墙体轮廓点集
         * @return std::vector<Event> 墙体关键点的属性
         */
        std::vector<Event> generateWallEventList(const cv::Mat &map, const Polygon &external_contour);

        /**
         * @brief 对轮廓的属性进行标记（障碍物、边界）
         * @param map 原始地图
         * @param event_list 初始化的属性
         * @return void
         */
        void allocateWallEventType(const cv::Mat &map, std::vector<Event> &event_list);
        void allocateObstacleEventType(const cv::Mat &map, std::vector<Event> &event_list);

        /**
         * @brief 初始化轮廓
         * @param polygon 轮廓
         * @param polygon_index 轮廓需要被标记的序号
         * @return std::vector<Event> 轮廓的属性
         */
        std::vector<Event> initializeEventList(const Polygon &polygon, int polygon_index);

        /**
         * @brief 对所有点集进行切片分类 （以x相同作为同一组数据）
         * @param wall_event_list 所有墙体特征点集
         * @param obstacle_event_list 所有障碍物特征点集
         * @return std::deque<std::deque<Event>> 分类好的点集序列
         */
        std::deque<std::deque<Event>> sliceListGenerator(const std::vector<Event> &wall_event_list, const std::vector<Event> &obstacle_event_list);

        /**
         * @brief 测试函数 画出障碍物关键点类型
         * @param map 原始地图
         * @param obstacle 障碍物轮廓点
         * @return void
         */
        void drawObstaclePointType(cv::Mat &map, const Polygon &obstacle);

        /**
         * @brief 测试函数 画出墙体关键点类型
         * @param map 原始地图
         * @param wall 墙体类型
         * @return void
         */
        void drawWallPointType(cv::Mat &map, const Polygon &wall);

        /**
         * @brief  滤除中间和处于初始状态的分组点集
         * @param slice 需要过滤的点集
         * @return std::deque<Event> 返回过滤后的点集
         */
        std::deque<Event> filterSlice(const std::deque<Event> &slice);

        /**
         * @brief 测试代码 显示障碍物 and 墙体的点集类型
         * @param map 地图
         * @param wall 墙体轮廓
         * @param obstacles 障碍物轮廓
         * @return void
         */
        void checkPointType(const cv::Mat &map, const Polygon &wall, const PolygonList &obstacles);

        /**
         * @brief 测试代码 显示分割区域轮廓
         * @param map 地图
         * @param cell_graph 区域轮廓点集
         * @return void
         */
        void checkGeneratedCells(const cv::Mat &map, const std::vector<CellNode> &cell_graph);

        /**
         * @brief 画出区域轮廓
         * @param map 底图
         * @param cell 轮廓
         * @param color 画图的颜色
         * @return void
         */
        void drawCells(cv::Mat &map, const CellNode &cell, cv::Scalar color = cv::Scalar(100, 100, 100));

        /**
         * @brief 寻找覆盖起点所对应的bcd分解轮廓id
         * @param cell_graph 分解得到的所有轮廓
         * @param point 起点
         * @return std::vector<int> 虽然返回的是Vector，但实际上只使用Vector.front()
         */
        std::vector<int> determineCellIndex(std::vector<CellNode> &cell_graph, const Point2D &point);

        /**
         * @brief 计算bcd分区轮廓的四个角点
         * @param cell 区域轮廓
         * @return std::vector<Point2D> 返回四个角点轮廓
         */
        std::vector<Point2D> computeCellCornerPoints(const CellNode &cell);

        /**
         * @brief 得到分解轮廓的访问顺序（深度优先遍历）
         * @param cell_graph 所有需要访问的轮廓
         * @param cell_index 初始轮廓
         * @param unvisited_counter 轮廓数量
         * @param path 轮廓的访问顺序（以引用的方式输出）
         */
        void walkThroughGraph(std::vector<CellNode> &cell_graph, int cell_index, int &unvisited_counter, std::deque<CellNode> &path);

        /**
         * @brief 得到区域访问顺序（实际上是调用 walkThroughGraph 函数得到 ）
         * @param cell_graph 所有需要访问的轮廓
         * @param first_cell_index 第一个访问的轮廓
         * @return std::deque<CellNode> 轮廓的访问顺序
         */
        std::deque<CellNode> getVisittingPath(std::vector<CellNode> &cell_graph, int first_cell_index);

        /**
         * @brief 初始化颜色队列
         * @param color_deque 以引用的形式输出颜色队列（输出）
         * @param repeat_times 同一个颜色重复次数
         * @return void
         */
        void initializeColorMap(std::deque<cv::Scalar> &color_deque, int repeat_times);

        /**
         * @brief 起点至起始轮廓第一个点 中规划的路径
         * @param cell 覆盖轮廓
         * @param start 起点
         * @param end 终点
         * @return std::deque<Point2D> 路径
         */
        std::deque<Point2D> walkInsideCell(CellNode cell, const Point2D &start, const Point2D &end);

        /**
         * @brief 计算连接路径
         * @param curr_exit 当前出口
         * @param next_entrance 下一个轮廓入口
         * @param corner_indicator 参考角点
         * @param curr_cell 当前轮廓
         * @param next_cell 下个要连接的轮廓
         * @return std::deque<std::deque<Point2D>> 计算得到的路径点
         */
        std::deque<std::deque<Point2D>> findLinkingPath(const Point2D &curr_exit, Point2D &next_entrance, int &corner_indicator, CellNode curr_cell, const CellNode &next_cell);

        /**
         * @brief 得到弓字切割路径
         * @param cell_graph 分割得到的所有轮廓
         * @param cell 当前需要规划的轮廓
         * @param corner_indicator 参考角点
         * @param robot_radius 规划半径
         * @return std::deque<Point2D> 输出弓字型路径
         */
        std::deque<Point2D> getBoustrophedonPath(std::vector<CellNode> &cell_graph, CellNode cell, int corner_indicator, int robot_radius);

        /**
         * @brief 寻找下个弓字区域的入口（起点）
         * @param curr_point 当前点
         * @param next_cell 下个区域轮廓
         * @param corner_indicator 参考角点
         * @return Point2D 下个弓字形 起点
         */
        Point2D findNextEntrance(const Point2D &curr_point, const CellNode &next_cell, int &corner_indicator);

        /**
         * @brief 计算整个区域的弓字形路径
         * @param cell_graph 所有分解得到的区域
         * @param start_point 起点
         * @param clean_distance 弓字形间距
         * @return std::deque<std::deque<Point2D>> 整个区域弓字形路径
         */
        std::deque<std::deque<Point2D>> staticPathPlanning(std::vector<CellNode> &cell_graph, const Point2D &start_point, double clean_distance);

        /**
         * @brief 打印所有弓字形路径
         * @param path 路径点集
         * @return void
         */
        void printPathNodes(const std::deque<std::deque<Point2D>> &path);

        /**
         * @brief 过滤弓字形路径（只做了一件事，去重）
         * @param raw_trajectory 原始路径
         * @return std::deque<Point2D> 过滤后的路径
         */
        std::deque<Point2D> filterTrajectory(const std::deque<std::deque<Point2D>> &raw_trajectory);

        /**
         * @brief 检测路径的一致性（是否有断点 或者 重复点）
         * @param path 弓字形路径
         * @return void
         */
        void checkPathConsistency(const std::deque<Point2D> &path);

        /**
         * @brief 可视化轨迹
         * @param original_map 原始地图
         * @param path 原始路径
         * @param robot_radius 机器人半径
         * @param vis_mode 显示模式
         * @param time_interval、 colors 跟显示颜色变化有关
         * @return void
         */
        void visualizeTrajectory(const cv::Mat &original_map, const std::deque<Point2D> &path, double robot_radius, int vis_mode, int time_interval = 10, int colors = 1530);

        /**
         * @brief 正则化容器的索引值
         * @param index 索引值
         * @param list_length 容器大小
         * @return int 正则化后的索引值
         */
        int wrappedIndex(int index, int list_length);

        /**
         * @brief  更新显示颜色（将颜色队列中的第一个元素放入最后一个）
         * @param  color_deque 颜色队列
         * @return void
         */
        void updateColor(std::deque<cv::Scalar> &color_deque);
    };
}

