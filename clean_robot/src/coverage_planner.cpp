/**
 * @author lijun
 * @date 2025-1-1
 * @brief Coverage Path Planning Using BCD Algorithm
 * 源代码是借鉴 Richey Huang  进行了少量了修改整理！
 * 站在前人的肩膀上可以走的更快！！！
 */
#include "coverage_planner.h"

// 是否打卡过程显示
#define Process_Visual 0

// 可视化显示模式
enum VisualizationMode
{
    PATH_MODE,
    ROBOT_MODE
};

namespace coverageplanner
{
    // 覆盖路径规划器
    std::deque<Point2D> CoveragePlanner::planner(const cv::Mat &clean_map, double robot_radius, double clean_distance , Point2D start)
    {
        if (clean_map.empty())
        {
            std::cout << "Error, clean map is empty!" << std::endl;
            return std::deque<Point2D>();
        }
        else
        {
            cv::Mat1b map = clean_map.clone();
            cv::threshold(map, map, 128, 255, cv::THRESH_BINARY);
            // 外边界轮廓
            std::vector<std::vector<cv::Point>> wall_contours;
            // 障碍物轮廓
            std::vector<std::vector<cv::Point>> obstacle_contours;
            // 提取并进行显示
            extractContours(map, wall_contours, obstacle_contours, robot_radius);
            if (Process_Visual)
            {
                showExtractedContours(map, wall_contours);
                showExtractedContours(map, obstacle_contours);
            }

            // 将障碍物轮廓、边界轮廓进行上采样处理
            PolygonList obstacles = constructObstacles(map, obstacle_contours);
            Polygon wall = constructWall(map, wall_contours.front());
            // 得到区域分解轮廓
            std::vector<CellNode> cell_graph = constructCellGraph(map, wall_contours, obstacle_contours, wall, obstacles);
            if (Process_Visual)
            {
                // checkPointType(map, wall, obstacles);
                std::cout << "cell_graph ";
                checkGeneratedCells(map, cell_graph);
            }


            // Point2D start = {25, 20};
            std::cout << "Start point: (" << start.x << ", " << start.y << ")" << std::endl;
            // staticPathPlanning(cell_graph, start, clean_distance);
            // std::cout << "Start point: (" << start.x << ", " << start.y << ")" << std::endl;
            std::deque<std::deque<Point2D>> original_planning_path = staticPathPlanning(cell_graph, start, clean_distance);
            if (Process_Visual)
            {
                printPathNodes(original_planning_path);
            }

            std::deque<Point2D> path = filterTrajectory(original_planning_path);
            // // 打印 path 的前10个元素
            // for (int i = 0; i < std::min(10, (int)path.size()); ++i)
            // {
            //     std::cout << "path[" << i << "]: (" << path[i].x << ", " << path[i].y << ")" << std::endl;
            // }
            if (0)
            {
                // checkPathConsistency(path);
                int time_interval = 20;
                visualizeTrajectory(clean_map, path, robot_radius, 1, time_interval);
            }
            std::cout << "path.size(): " << path.size() << std::endl;
            return path;
        }
    }

    void CoveragePlanner::extractContours(const cv::Mat &original_map, std::vector<std::vector<cv::Point>> &wall_contours, std::vector<std::vector<cv::Point>> &obstacle_contours, double robot_radius)
    {

        cv::Mat erode_map = original_map.clone();
        cv::threshold(erode_map, erode_map, 128, 255, cv::THRESH_BINARY);
        // cv::namedWindow("erode_map1", cv::WINDOW_NORMAL);
        // cv::resizeWindow("erode_map1", 1200, 900);
        // cv::imshow("erode_map1", erode_map);
        // cv::waitKey(5000);
        if (robot_radius != 0)
        {
            cv::Mat erod_struct = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size((robot_radius * 2), (robot_radius * 2)));
            cv::erode(erode_map, erode_map, erod_struct);
        }
        // cv::namedWindow("erode_map", cv::WINDOW_NORMAL);
        // cv::resizeWindow("erode_map", 1200, 900);
        // cv::imshow("erode_map", erode_map);
        // cv::waitKey(5000);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(erode_map, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        std::vector<int> wall_cnt_indices(contours.size());
        std::iota(wall_cnt_indices.begin(), wall_cnt_indices.end(), 0);
        std::sort(wall_cnt_indices.begin(), wall_cnt_indices.end(), [&contours](int lhs, int rhs)
                  { return cv::contourArea(contours[lhs]) > cv::contourArea(contours[rhs]); });

        // 找到外轮廓(外轮廓最大)
        std::vector<cv::Point> raw_wall_contour = contours[wall_cnt_indices.front()];
        wall_contours = {raw_wall_contour};

        // 寻找障碍物轮廓
        cv::Mat mask = cv::Mat(erode_map.size(), erode_map.type(), 255);
        cv::fillPoly(mask, wall_contours, 0);
        cv::Mat base = erode_map.clone();
        base += mask;
        // cv::THRESH_BINARY_INV 阈值类型反转，大于等于128会变成黑色，小于128会变成白色
        cv::threshold(base, base, 128, 255, cv::THRESH_BINARY_INV);
        cv::findContours(base, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        obstacle_contours = contours;

        // 做多边形的拟合
        std::vector<cv::Point> processed_wall_contour;
        cv::approxPolyDP(cv::Mat(wall_contours.front()), processed_wall_contour, 1, true);
        wall_contours = {processed_wall_contour};

        std::vector<std::vector<cv::Point>> processed_obstacle_contours(obstacle_contours.size());
        for (int i = 0; i < obstacle_contours.size(); i++)
        {
            cv::approxPolyDP(cv::Mat(obstacle_contours[i]), processed_obstacle_contours[i], 1, true);
        }
        obstacle_contours = processed_obstacle_contours;
    }

    // 显示提取到的轮廓
    void CoveragePlanner::showExtractedContours(const cv::Mat &map, const std::vector<std::vector<cv::Point>> &contours)
    {
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::resizeWindow("map", 1200, 900);
        cv::Mat3b canvas = cv::Mat3b(map.size(), CV_8U);
        canvas.setTo(cv::Scalar(0, 0, 0));
        for (int i = 0; i <= contours.size() - 1; i++)
        {
            cv::drawContours(canvas, contours, i, cv::Scalar(255, 0, 0));
            cv::imshow("map", canvas);
            cv::waitKey(500);
        }
        cv::destroyWindow("map");
    }

    // 将障碍物点集密集化
    PolygonList CoveragePlanner::constructObstacles(const cv::Mat &original_map, const std::vector<std::vector<cv::Point>> &obstacle_contours)
    {
        PolygonList obstacles;
        for (const auto &obstacle_contour : obstacle_contours)
        {
            Polygon obstacle;
            int obs_size = obstacle_contour.size();
            for (int j = 0; j < obs_size; j++)
            {
                cv::LineIterator line(original_map, obstacle_contour[j], obstacle_contour[(j + 1) % obs_size]);
                for (int k = 0; k < line.count - 1; k++)
                {
                    obstacle.emplace_back(Point2D(line.pos().x, line.pos().y));
                    line++;
                }
            }

            obstacles.emplace_back(obstacle);
        }
        return obstacles;
    }

    // 将外轮廓点集密集化
    Polygon CoveragePlanner::constructWall(const cv::Mat &original_map, std::vector<cv::Point> &wall_contour)
    {
        Polygon wall;
        if (!wall_contour.empty())
        {
            int wall_size = wall_contour.size();
            for (int i = 0; i < wall_size; i++)
            {
                cv::LineIterator line(original_map, wall_contour[i], wall_contour[(i + 1) % wall_size]);
                for (int j = 0; j < line.count - 1; j++)
                {
                    wall.emplace_back(Point2D(line.pos().x, line.pos().y));
                    line++;
                }
            }
            return wall;
        }
        else
        {
            // 外轮廓为空的情况下使用地图边界
            std::vector<cv::Point> default_wall_contour = {cv::Point(0, 0), cv::Point(0, original_map.rows - 1), cv::Point(original_map.cols - 1, original_map.rows - 1), cv::Point(original_map.cols - 1, 0)};
            std::vector<std::vector<cv::Point>> default_wall_contours = {default_wall_contour};
            wall = constructObstacles(original_map, default_wall_contours).front();
            for (const auto &point : wall)
            {
                wall_contour.emplace_back(cv::Point(point.x, point.y));
            }
            return wall;
        }
    }

    std::vector<CellNode> CoveragePlanner::constructCellGraph(const cv::Mat &original_map, const std::vector<std::vector<cv::Point>> &wall_contours, const std::vector<std::vector<cv::Point>> &obstacle_contours, const Polygon &wall, const PolygonList &obstacles)
    {
        cv::Mat3b map = cv::Mat3b(original_map.size());
        map.setTo(cv::Scalar(0, 0, 0));
        cv::fillPoly(map, wall_contours, cv::Scalar(255, 255, 255));
        cv::fillPoly(map, obstacle_contours, cv::Scalar(0, 0, 0));

        std::vector<Event> obstacle_event_list = generateObstacleEventList(map, obstacles);
        std::vector<Event> wall_event_list = generateWallEventList(map, wall);

        std::deque<std::deque<Event>> slice_list = sliceListGenerator(wall_event_list, obstacle_event_list);

        std::vector<CellNode> cell_graph;
        executeCellDecomposition(cell_graph, slice_list);
        std::cout << "cell_graph.size(): " << cell_graph.size() << std::endl;
        return cell_graph;
    }

    // void CoveragePlanner::checkPointType(const cv::Mat &map, const Polygon &wall, const PolygonList &obstacles)
    // {
    //     cv::namedWindow("map", cv::WINDOW_NORMAL);
    //     cv::resizeWindow("map", 1200, 900);
    //     cv::Mat vis_map = map.clone();
    //     cv::cvtColor(vis_map, vis_map, cv::COLOR_GRAY2BGR);

    //     for (const auto &obstacle : obstacles)
    //     {
    //         drawObstaclePointType(vis_map, obstacle);
    //         cv::imshow("map", vis_map);
    //         cv::waitKey(0);
    //         std::cout << std::endl;
    //     }

    //     {
    //         cv::Mat frame = vis_map.clone();
    //         drawWallPointType(frame, wall);
    //         cv::imshow("map", frame);
    //         cv::waitKey(0);

    //     }
    // }
    void CoveragePlanner::checkPointType(const cv::Mat &map, const Polygon &wall, const PolygonList &obstacles)
    {
        cv::Mat vis_map = map.clone();
        cv::cvtColor(vis_map, vis_map, cv::COLOR_GRAY2BGR);

        // 只创建一次窗口并固定大小
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::resizeWindow("map", 1200, 900);

        for (const auto &obstacle : obstacles)
        {
            cv::Mat frame = vis_map.clone();
            drawObstaclePointType(frame, obstacle);
            cv::imshow("map", frame);

            cv::waitKey(500);
        }

        {
            cv::Mat frame = vis_map.clone();
            drawWallPointType(frame, wall);
            cv::imshow("map", frame);

            cv::waitKey(500);
        }

        cv::destroyWindow("map");
    }

    void CoveragePlanner::checkGeneratedCells(const cv::Mat &map, const std::vector<CellNode> &cell_graph)
    {
        cv::Mat vis_map = map.clone();
        cv::cvtColor(vis_map, vis_map, cv::COLOR_GRAY2BGR);

        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::resizeWindow("map", 1200, 900);
        for (const auto &cell : cell_graph)
        {
            drawCells(vis_map, cell, cv::Scalar(255, 0, 255));
            cv::imshow("map", vis_map);
            cv::waitKey(500);
        }
        cv::destroyWindow("map");
    }

    std::deque<std::deque<Point2D>> CoveragePlanner::staticPathPlanning(std::vector<CellNode> &cell_graph, const Point2D &start_point, double clean_distance)
    {

        // 寻找起始轮廓id
        int start_cell_index = determineCellIndex(cell_graph, start_point).front();
        // 起点至起始轮廓第一个点 规划的路径
        std::deque<Point2D> init_path = walkInsideCell(cell_graph[start_cell_index], start_point, computeCellCornerPoints(cell_graph[start_cell_index])[TOP_LEFT]);
        std::deque<Point2D> local_path;
        local_path.assign(init_path.begin(), init_path.end());

        // 计算所有轮廓的访问顺序
        std::deque<CellNode> cell_path = getVisittingPath(cell_graph, start_cell_index);

        std::deque<Point2D> inner_path;
        std::deque<std::deque<Point2D>> link_path;
        Point2D curr_exit;
        Point2D next_entrance;

        std::deque<int> return_cell_path;
        std::deque<Point2D> return_path;

        int corner_indicator = TOP_LEFT;
        std::deque<std::deque<Point2D>> global_path;

        for (int i = 0; i < cell_path.size(); i++)
        {
            inner_path = getBoustrophedonPath(cell_graph, cell_path[i], corner_indicator, clean_distance);
            local_path.insert(local_path.end(), inner_path.begin(), inner_path.end());
            // 标记为清洁过
            cell_graph[cell_path[i].cellIndex].isCleaned = true;
            if (i < (cell_path.size() - 1))
            {
                curr_exit = inner_path.back();
                // 寻找下一个弓字形入口（起点）
                next_entrance = findNextEntrance(curr_exit, cell_path[i + 1], corner_indicator);
                // 计算当前点（出口） 至 弓字形入口 的 连接路径（跟walkInsideCell功能一致）
                link_path = findLinkingPath(curr_exit, next_entrance, corner_indicator, cell_path[i], cell_path[i + 1]);
                local_path.insert(local_path.end(), link_path.front().begin(), link_path.front().end());
                global_path.emplace_back(local_path);
                local_path.clear();
                local_path.insert(local_path.end(), link_path.back().begin(), link_path.back().end());
            }
        }
        global_path.emplace_back(local_path);

        return global_path;
    }

    std::deque<Point2D> CoveragePlanner::filterTrajectory(const std::deque<std::deque<Point2D>> &raw_trajectory)
    {
        std::deque<Point2D> trajectory;

        for (const auto &sub_trajectory : raw_trajectory)
        {
            for (const auto &position : sub_trajectory)
            {
                if (!trajectory.empty())
                {
                    if (position != trajectory.back())
                    {
                        trajectory.emplace_back(position);
                    }
                }
                else
                {
                    trajectory.emplace_back(position);
                }
            }
        }

        return trajectory;
    }

    void CoveragePlanner::checkPathConsistency(const std::deque<Point2D> &path)
    {
        // 断点
        int breakpoints = 0;
        // 重复点
        int duplicates = 0;

        for (int i = 1; i < path.size(); i++)
        {
            if (std::abs(path[wrappedIndex((i - 1), path.size())].x - path[i].x) > 1 || std::abs(path[wrappedIndex((i - 1), path.size())].y - path[i].y) > 1)
            {
                breakpoints++;
                std::cout << "break points :" << path[wrappedIndex((i - 1), path.size())].x << ", " << path[wrappedIndex((i - 1), path.size())].y
                          << "---->" << path[i].x << ", " << path[i].y << std::endl;
            }
            if (path[wrappedIndex((i - 1), path.size())] == path[i])
            {
                duplicates++;
            }
        }
        std::cout << "breakpoints: " << breakpoints << std::endl;
        std::cout << "duplicates: " << duplicates << std::endl;
    }

    void CoveragePlanner::visualizeTrajectory(const cv::Mat &original_map, const std::deque<Point2D> &path, double robot_radius, int vis_mode, int time_interval, int colors)
    {
        cv::Mat3b vis_map;
        cv::cvtColor(original_map, vis_map, cv::COLOR_GRAY2BGR);

        cv::namedWindow("map", cv::WINDOW_NORMAL);

        std::deque<cv::Scalar> color_deque;
        int color_repeated_times = path.size() / colors + 1;
        initializeColorMap(color_deque, color_repeated_times);

        switch (vis_mode)
        {
        case PATH_MODE:
            vis_map.at<cv::Vec3b>(path.front().y, path.front().x) = cv::Vec3b(uchar(color_deque.front()[0]), uchar(color_deque.front()[1]), uchar(color_deque.front()[2]));
            cv::imshow("map", vis_map);
            cv::waitKey(0);

            for (const auto &position : path)
            {
                vis_map.at<cv::Vec3b>(position.y, position.x) = cv::Vec3b(uchar(color_deque.front()[0]), uchar(color_deque.front()[1]), uchar(color_deque.front()[2]));
                updateColor(color_deque);
                cv::imshow("map", vis_map);
                cv::waitKey(time_interval);
            }
            break;
        case ROBOT_MODE:
            cv::circle(vis_map, cv::Point(path.front().x, path.front().y), robot_radius, cv::Scalar(255, 204, 153), -1);
            // cv::resizeWindow("map", 1200, 900);
            // cv::imshow("map", vis_map);
            // cv::waitKey(0);

            for (const auto &position : path)
            {
                cv::circle(vis_map, cv::Point(position.x, position.y), robot_radius, cv::Scalar(255, 204, 153), -1);
                cv::resizeWindow("map", 1200, 900);
                cv::imshow("map", vis_map);
                cv::waitKey(time_interval);

                cv::circle(vis_map, cv::Point(position.x, position.y), robot_radius, cv::Scalar(255, 229, 204), -1);
            }
            break;
        default:
            break;
        }

        cv::waitKey(0);
    }

    /////////////////////////////////
    int CoveragePlanner::wrappedIndex(int index, int list_length)
    {
        int wrapped_index = (index % list_length + list_length) % list_length;
        return wrapped_index;
    }

    std::vector<int> CoveragePlanner::determineCellIndex(std::vector<CellNode> &cell_graph, const Point2D &point)
    {
        std::vector<int> cell_index;

        for (int i = 0; i < cell_graph.size(); i++)
        {
            for (int j = 0; j < cell_graph[i].ceiling.size(); j++)
            {
                if (point.x == cell_graph[i].ceiling[j].x && point.y >= cell_graph[i].ceiling[j].y && point.y <= cell_graph[i].floor[j].y)
                {
                    cell_index.emplace_back(int(i));
                }
            }
        }
        return cell_index;
    }

    std::deque<Point2D> CoveragePlanner::walkInsideCell(CellNode cell, const Point2D &start, const Point2D &end)
    {
        std::deque<Point2D> inner_path = {start};
        int start_ceiling_index_offset = start.x - cell.ceiling.front().x;
        int first_ceiling_delta_y = cell.ceiling[start_ceiling_index_offset].y - start.y;
        int end_ceiling_index_offset = end.x - cell.ceiling.front().x;
        int second_ceiling_delta_y = end.y - cell.ceiling[end_ceiling_index_offset].y;

        int start_floor_index_offset = start.x - cell.floor.front().x;
        int first_floor_delta_y = cell.floor[start_floor_index_offset].y - start.y;
        int end_floor_index_offset = end.x - cell.floor.front().x;
        int second_floor_delta_y = end.y - cell.floor[end_floor_index_offset].y;

        // 选择是从天花板开始还是底部开始
        if ((abs(first_ceiling_delta_y) + abs(second_ceiling_delta_y)) < (abs(first_floor_delta_y) + abs(second_floor_delta_y))) // to ceiling
        {
            int first_increment_y = 0;
            if (first_ceiling_delta_y != 0)
            {
                first_increment_y = first_ceiling_delta_y / abs(first_ceiling_delta_y);

                for (int i = 1; i <= abs(first_ceiling_delta_y); i++)
                {
                    inner_path.emplace_back(Point2D(start.x, start.y + (first_increment_y * i)));
                }
            }

            int delta_x = cell.ceiling[end_ceiling_index_offset].x - cell.ceiling[start_ceiling_index_offset].x;
            int increment_x = 0;
            if (delta_x != 0)
            {
                increment_x = delta_x / abs(delta_x);
            }
            for (int i = 0; i < abs(delta_x); i++)
            {
                // 提前转
                if ((cell.ceiling[start_ceiling_index_offset + increment_x * (i + 1)].y - cell.ceiling[start_ceiling_index_offset + increment_x * (i)].y >= 2) && (i + 1 <= abs(delta_x)) && (i <= abs(delta_x)))
                {
                    int delta = cell.ceiling[start_ceiling_index_offset + increment_x * (i + 1)].y - cell.ceiling[start_ceiling_index_offset + increment_x * (i)].y;
                    int increment = delta / abs(delta);
                    for (int j = 0; j <= abs(delta); j++)
                    {
                        inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset + increment_x * i].x, cell.ceiling[start_ceiling_index_offset + increment_x * i].y + increment * (j)));
                    }
                }
                // 滞后转
                else if ((cell.ceiling[start_ceiling_index_offset + increment_x * (i)].y - cell.ceiling[start_ceiling_index_offset + increment_x * (i + 1)].y >= 2) && (i <= abs(delta_x)) && (i + 1 <= abs(delta_x)))
                {
                    inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset + increment_x * (i)]);

                    int delta = cell.ceiling[start_ceiling_index_offset + increment_x * (i + 1)].y - cell.ceiling[start_ceiling_index_offset + increment_x * (i)].y;

                    int increment = delta / abs(delta);
                    for (int k = 0; k <= abs(delta); k++)
                    {
                        inner_path.emplace_back(Point2D(cell.ceiling[start_ceiling_index_offset + increment_x * (i + 1)].x, cell.ceiling[start_ceiling_index_offset + increment_x * (i + 1)].y + abs(delta) + increment * (k)));
                    }
                }
                else
                {
                    inner_path.emplace_back(cell.ceiling[start_ceiling_index_offset + (increment_x * i)]);
                }
            }

            int second_increment_y = 0;
            if (second_ceiling_delta_y != 0)
            {
                second_increment_y = second_ceiling_delta_y / abs(second_ceiling_delta_y);

                for (int i = 1; i <= abs(second_ceiling_delta_y); i++)
                {
                    inner_path.emplace_back(Point2D(cell.ceiling[end_ceiling_index_offset].x, cell.ceiling[end_ceiling_index_offset].y + (second_increment_y * i)));
                }
            }
        }
        else // to floor
        {
            int first_increment_y = 0;
            if (first_floor_delta_y != 0)
            {
                first_increment_y = first_floor_delta_y / abs(first_floor_delta_y);

                for (int i = 1; i <= abs(first_floor_delta_y); i++)
                {
                    inner_path.emplace_back(Point2D(start.x, start.y + (first_increment_y * i)));
                }
            }

            int delta_x = cell.floor[end_floor_index_offset].x - cell.floor[start_floor_index_offset].x;
            int increment_x = 0;
            if (delta_x != 0)
            {
                increment_x = delta_x / abs(delta_x);
            }
            for (int i = 0; i < abs(delta_x); i++)
            {
                // 提前转
                if ((cell.floor[start_floor_index_offset + increment_x * (i)].y - cell.floor[start_floor_index_offset + increment_x * (i + 1)].y >= 2) && (i <= abs(delta_x)) && (i + 1 <= abs(delta_x)))
                {
                    int delta = cell.floor[start_floor_index_offset + increment_x * (i + 1)].y - cell.floor[start_floor_index_offset + increment_x * (i)].y;
                    int increment = delta / abs(delta);
                    for (int j = 0; j <= abs(delta); j++)
                    {
                        inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset + increment_x * (i)].x, cell.floor[start_floor_index_offset + increment_x * (i)].y + increment * (j)));
                    }
                }
                // 滞后转
                else if ((cell.floor[start_floor_index_offset + increment_x * (i + 1)].y - cell.floor[start_floor_index_offset + increment_x * (i)].y >= 2) && (i + 1 <= abs(delta_x)) && (i <= abs(delta_x)))
                {
                    inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset + increment_x * (i)].x, cell.floor[start_floor_index_offset + increment_x * (i)].y));

                    int delta = cell.floor[start_floor_index_offset + increment_x * (i + 1)].y - cell.floor[start_floor_index_offset + increment_x * (i)].y;

                    int increment = delta / abs(delta);
                    for (int k = 0; k <= abs(delta); k++)
                    {
                        inner_path.emplace_back(Point2D(cell.floor[start_floor_index_offset + increment_x * (i + 1)].x, cell.floor[start_floor_index_offset + increment_x * (i + 1)].y - abs(delta) + increment * (k)));
                    }
                }
                else
                {
                    inner_path.emplace_back(cell.floor[start_floor_index_offset + (increment_x * i)]);
                }
            }

            int second_increment_y = 0;
            if (second_floor_delta_y != 0)
            {
                second_increment_y = second_floor_delta_y / abs(second_floor_delta_y);

                for (int i = 1; i <= abs(second_floor_delta_y); i++)
                {
                    inner_path.emplace_back(Point2D(cell.floor[end_floor_index_offset].x, cell.floor[end_floor_index_offset].y + (second_increment_y * i)));
                }
            }
        }
        return inner_path;
    }

    void CoveragePlanner::walkThroughGraph(std::vector<CellNode> &cell_graph, int cell_index, int &unvisited_counter, std::deque<CellNode> &path)
    {
        if (!cell_graph[cell_index].isVisited)
        {
            cell_graph[cell_index].isVisited = true;
            unvisited_counter--;
        }
        path.emplace_front(cell_graph[cell_index]);

        CellNode neighbor;
        int neighbor_idx = INT_MAX;

        for (int i = 0; i < cell_graph[cell_index].neighbor_indices.size(); i++)
        {
            neighbor = cell_graph[cell_graph[cell_index].neighbor_indices[i]];
            neighbor_idx = cell_graph[cell_index].neighbor_indices[i];
            if (!neighbor.isVisited)
            {
                break;
            }
        }

        if (!neighbor.isVisited) // unvisited neighbor found
        {
            cell_graph[neighbor_idx].parentIndex = cell_graph[cell_index].cellIndex;
            walkThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
        }
        else // unvisited neighbor not found
        {

            if (cell_graph[cell_index].parentIndex == INT_MAX) // cannot go on back-tracking
            {
                return;
            }
            else if (unvisited_counter == 0)
            {
                return;
            }
            else
            {
                walkThroughGraph(cell_graph, cell_graph[cell_index].parentIndex, unvisited_counter, path);
            }
        }
    }

    std::deque<CellNode> CoveragePlanner::getVisittingPath(std::vector<CellNode> &cell_graph, int first_cell_index)
    {
        std::deque<CellNode> visitting_path;

        if (cell_graph.size() == 1)
        {
            visitting_path.emplace_back(cell_graph.front());
        }
        else
        {
            int unvisited_counter = cell_graph.size();
            walkThroughGraph(cell_graph, first_cell_index, unvisited_counter, visitting_path);
            std::reverse(visitting_path.begin(), visitting_path.end());
        }

        return visitting_path;
    }

    void CoveragePlanner::drawCells(cv::Mat &map, const CellNode &cell, cv::Scalar color)
    {
        std::cout << "cell " << cell.cellIndex << ": " << std::endl;
        std::cout << "cell's ceiling points: " << cell.ceiling.size() << std::endl;
        std::cout << "cell's floor points: " << cell.floor.size() << std::endl;

        for (const auto &ceiling_point : cell.ceiling)
        {
            map.at<cv::Vec3b>(ceiling_point.y, ceiling_point.x) = cv::Vec3b(255, 0, 0);
        }

        for (const auto &floor_point : cell.floor)
        {
            map.at<cv::Vec3b>(floor_point.y, floor_point.x) = cv::Vec3b(255, 0, 0);
        }

        cv::line(map, cv::Point(cell.ceiling.front().x, cell.ceiling.front().y), cv::Point(cell.floor.front().x, cell.floor.front().y), color);
        cv::line(map, cv::Point(cell.ceiling.back().x, cell.ceiling.back().y), cv::Point(cell.floor.back().x, cell.floor.back().y), color);
    }

    void CoveragePlanner::updateColor(std::deque<cv::Scalar> &color_deque)
    {
        cv::Scalar color = color_deque.front();
        color_deque.pop_front();
        color_deque.emplace_back(color);
    }

    std::vector<Point2D> CoveragePlanner::computeCellCornerPoints(const CellNode &cell)
    {
        // 顶左 顶右
        Point2D topleft = cell.ceiling.front();
        Point2D topright = cell.ceiling.back();
        // 底左 底右
        Point2D bottomleft = cell.floor.front();
        Point2D bottomright = cell.floor.back();
        // 按照TOPLEFT、BOTTOMLEFT、BOTTOMRIGHT、TOPRIGHT的顺序储存corner points（逆时针）
        std::vector<Point2D> corner_points = {topleft, bottomleft, bottomright, topright};
        return corner_points;
    }

    Point2D CoveragePlanner::findNextEntrance(const Point2D &curr_point, const CellNode &next_cell, int &corner_indicator)
    {
        Point2D next_entrance;

        int front_x = next_cell.ceiling.front().x;
        int back_x = next_cell.ceiling.back().x;

        std::vector<Point2D> corner_points = computeCellCornerPoints(next_cell);

        if (abs(curr_point.x - front_x) < abs(curr_point.x - back_x))
        {
            if (abs(curr_point.y - next_cell.ceiling.front().y) < abs(curr_point.y - next_cell.floor.front().y))
            {
                next_entrance = corner_points[TOP_LEFT];
                corner_indicator = TOP_LEFT;
            }
            else
            {
                next_entrance = corner_points[BOTTOM_LEFT];
                corner_indicator = BOTTOM_LEFT;
            }
        }
        else
        {
            if (abs(curr_point.y - next_cell.ceiling.back().y) < abs(curr_point.y - next_cell.floor.back().y))
            {
                next_entrance = corner_points[TOP_RIGHT];
                corner_indicator = TOP_RIGHT;
            }
            else
            {
                next_entrance = corner_points[BOTTOM_RIGHT];
                corner_indicator = BOTTOM_RIGHT;
            }
        }

        return next_entrance;
    }

    std::deque<std::deque<Point2D>> CoveragePlanner::findLinkingPath(const Point2D &curr_exit, Point2D &next_entrance, int &corner_indicator, CellNode curr_cell, const CellNode &next_cell)
    {
        std::deque<std::deque<Point2D>> path;
        std::deque<Point2D> path_in_curr_cell;
        std::deque<Point2D> path_in_next_cell;

        int exit_corner_indicator = INT_MAX;
        Point2D exit = findNextEntrance(next_entrance, curr_cell, exit_corner_indicator);
        path_in_curr_cell = walkInsideCell(curr_cell, curr_exit, exit);

        next_entrance = findNextEntrance(exit, next_cell, corner_indicator);

        int delta_x = next_entrance.x - exit.x;
        int delta_y = next_entrance.y - exit.y;

        int increment_x = 0;
        int increment_y = 0;

        if (delta_x != 0)
        {
            increment_x = delta_x / std::abs(delta_x);
        }
        if (delta_y != 0)
        {
            increment_y = delta_y / std::abs(delta_y);
        }

        int upper_bound = INT_MIN;
        int lower_bound = INT_MAX;

        if (exit.x >= curr_cell.ceiling.back().x)
        {
            upper_bound = curr_cell.ceiling.back().y;
            lower_bound = curr_cell.floor.back().y;
        }
        if (exit.x <= curr_cell.ceiling.front().x)
        {
            upper_bound = curr_cell.ceiling.front().y;
            lower_bound = curr_cell.floor.front().y;
        }

        if ((next_entrance.y >= upper_bound) && (next_entrance.y <= lower_bound))
        {
            for (int y = exit.y; y != next_entrance.y; y += increment_y)
            {
                path_in_curr_cell.emplace_back(Point2D(exit.x, y));
            }
            for (int x = exit.x; x != next_entrance.x; x += increment_x)
            {
                path_in_curr_cell.emplace_back(Point2D(x, next_entrance.y));
            }
        }
        else
        {
            for (int x = exit.x; x != next_entrance.x; x += increment_x)
            {
                path_in_curr_cell.emplace_back(Point2D(x, exit.y));
            }
            for (int y = exit.y; y != next_entrance.y; y += increment_y)
            {
                path_in_next_cell.emplace_back(Point2D(next_entrance.x, y));
            }
        }

        path = {path_in_curr_cell, path_in_next_cell};

        return path;
    }

    void CoveragePlanner::initializeColorMap(std::deque<cv::Scalar> &color_deque, int repeat_times)
    {
        for (int i = 0; i <= 255; i++)
        {
            for (int j = 0; j < repeat_times; j++)
            {
                color_deque.emplace_back(cv::Scalar(0, i, 255));
            }
        }

        for (int i = 254; i >= 0; i--)
        {
            for (int j = 0; j < repeat_times; j++)
            {
                color_deque.emplace_back(cv::Scalar(0, 255, i));
            }
        }

        for (int i = 1; i <= 255; i++)
        {
            for (int j = 0; j < repeat_times; j++)
            {
                color_deque.emplace_back(cv::Scalar(i, 255, 0));
            }
        }

        for (int i = 254; i >= 0; i--)
        {
            for (int j = 0; j < repeat_times; j++)
            {
                color_deque.emplace_back(cv::Scalar(255, i, 0));
            }
        }

        for (int i = 1; i <= 255; i++)
        {
            for (int j = 0; j < repeat_times; j++)
            {
                color_deque.emplace_back(cv::Scalar(255, 0, i));
            }
        }

        for (int i = 254; i >= 1; i--)
        {
            for (int j = 0; j < repeat_times; j++)
            {
                color_deque.emplace_back(cv::Scalar(i, 0, 255));
            }
        }
    }

    std::deque<Point2D> CoveragePlanner::getBoustrophedonPath(std::vector<CellNode> &cell_graph, CellNode cell, int corner_indicator, int robot_radius)
    {
        int delta, increment;

        std::deque<Point2D> path;

        std::vector<Point2D> corner_points = computeCellCornerPoints(cell);

        std::vector<Point2D> ceiling, floor;
        ceiling.assign(cell.ceiling.begin(), cell.ceiling.end());
        floor.assign(cell.floor.begin(), cell.floor.end());

        if (cell_graph[cell.cellIndex].isCleaned)
        {
            if (corner_indicator == TOP_LEFT)
            {
                path.emplace_back(corner_points[TOP_LEFT]);
            }
            if (corner_indicator == TOP_RIGHT)
            {
                path.emplace_back(corner_points[TOP_RIGHT]);
            }
            if (corner_indicator == BOTTOM_LEFT)
            {
                path.emplace_back(corner_points[BOTTOM_LEFT]);
            }
            if (corner_indicator == BOTTOM_RIGHT)
            {
                path.emplace_back(corner_points[BOTTOM_RIGHT]);
            }
        }
        else
        {
            if (corner_indicator == TOP_LEFT)
            {
                int x, y, y_start, y_end;
                bool reverse = false;

                for (int i = 0; i < ceiling.size(); i = i + (robot_radius + 1))
                {
                    x = ceiling[i].x;

                    if (!reverse)
                    {
                        y_start = ceiling[i].y;
                        y_end = floor[i].y;

                        for (y = y_start; y <= y_end; y++)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(floor[i + 1].y - floor[i].y) >= 2) && (i + 1 < floor.size()))
                        {
                            delta = floor[i + 1].y - floor[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(floor[i].x, floor[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着floor从左往右
                                if (x + j >= floor.back().x)
                                {
                                    i = i - (robot_radius - (j - 1));
                                    break;
                                }

                                // 提前转
                                else if ((floor[i + (j)].y - floor[i + (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    delta = floor[i + (j + 1)].y - floor[i + (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i + (j)].x, floor[i + (j)].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((floor[i + (j + 1)].y - floor[i + (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    path.emplace_back(Point2D(floor[i + (j)].x, floor[i + (j)].y));

                                    delta = floor[i + (j + 1)].y - floor[i + (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i + (j + 1)].x, cell.floor[i + (j + 1)].y - abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(floor[i + (j)]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                    else
                    {
                        y_start = floor[i].y;
                        y_end = ceiling[i].y;

                        for (y = y_start; y >= y_end; y--)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(ceiling[i + 1].y - ceiling[i].y) >= 2) && (i + 1 < ceiling.size()))
                        {
                            delta = ceiling[i + 1].y - ceiling[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着ceiling从左往右
                                if (x + j >= ceiling.back().x)
                                {
                                    i = i - (robot_radius - (j - 1));
                                    break;
                                }

                                // 提前转
                                else if ((ceiling[i + (j + 1)].y - ceiling[i + (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i + j].x, ceiling[i + j].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((ceiling[i + (j)].y - ceiling[i + (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    path.emplace_back(ceiling[i + (j)]);

                                    delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i + (j + 1)].x, ceiling[i + (j + 1)].y + abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(ceiling[i + j]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                }
            }

            if (corner_indicator == TOP_RIGHT)
            {
                int x = 0, y = 0, y_start = 0, y_end = 0;
                bool reverse = false;

                for (int i = ceiling.size() - 1; i >= 0; i = i - (robot_radius + 1))
                {
                    x = ceiling[i].x;

                    if (!reverse)
                    {
                        y_start = ceiling[i].y;
                        y_end = floor[i].y;

                        for (y = y_start; y <= y_end; y++)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(floor[i - 1].y - floor[i].y) >= 2) && (i - 1 >= 0))
                        {
                            delta = floor[i - 1].y - floor[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(floor[i].x, floor[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着floor从右往左
                                if (x - j <= floor.front().x)
                                {
                                    i = i + (robot_radius - (j - 1));
                                    break;
                                }
                                // 提前转
                                else if ((floor[i - (j)].y - floor[i - (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    delta = floor[i - (j + 1)].y - floor[i - (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i - (j)].x, floor[i - (j)].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((floor[i - (j + 1)].y - floor[i - (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    path.emplace_back(Point2D(floor[i - (j)].x, floor[i - (j)].y));

                                    delta = floor[i - (j + 1)].y - floor[i - (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i - (j + 1)].x, cell.floor[i - (j + 1)].y - abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(floor[i - (j)]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                    else
                    {
                        y_start = floor[i].y;
                        y_end = ceiling[i].y;

                        for (y = y_start; y >= y_end; y--)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(ceiling[i - 1].y - ceiling[i].y) >= 2) && (i - 1 >= 0))
                        {
                            delta = ceiling[i - 1].y - ceiling[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着ceiling从右往左
                                if (x - j <= ceiling.front().x)
                                {
                                    i = i + (robot_radius - (j - 1));
                                    break;
                                }
                                // 提前转
                                else if ((ceiling[i - (j + 1)].y - ceiling[i - (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i - j].x, ceiling[i - j].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((ceiling[i - (j)].y - ceiling[i - (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    path.emplace_back(ceiling[i - (j)]);

                                    delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i - (j + 1)].x, ceiling[i - (j + 1)].y + abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(ceiling[i - j]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                }
            }

            if (corner_indicator == BOTTOM_LEFT)
            {
                int x = 0, y = 0, y_start = 0, y_end = 0;
                bool reverse = false;

                for (int i = 0; i < ceiling.size(); i = i + (robot_radius + 1))
                {
                    x = ceiling[i].x;

                    if (!reverse)
                    {
                        y_start = floor[i].y;
                        y_end = ceiling[i].y;

                        for (y = y_start; y >= y_end; y--)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(ceiling[i + 1].y - ceiling[i].y) >= 2) && (i + 1 < ceiling.size()))
                        {
                            delta = ceiling[i + 1].y - ceiling[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着ceiling从左往右
                                if (x + j >= ceiling.back().x)
                                {
                                    i = i - (robot_radius - (j - 1));
                                    break;
                                }
                                // 提前转
                                else if ((ceiling[i + (j + 1)].y - ceiling[i + (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i + j].x, ceiling[i + j].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((ceiling[i + (j)].y - ceiling[i + (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    path.emplace_back(ceiling[i + (j)]);

                                    delta = ceiling[i + (j + 1)].y - ceiling[i + (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i + (j + 1)].x, ceiling[i + (j + 1)].y + abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(ceiling[i + j]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                    else
                    {
                        y_start = ceiling[i].y;
                        y_end = floor[i].y;

                        for (y = y_start; y <= y_end; y++)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(floor[i + 1].y - floor[i].y) >= 2) && (i + 1 < floor.size()))
                        {
                            delta = floor[i + 1].y - floor[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(floor[i].x, floor[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着floor从左往右
                                if (x + j >= floor.back().x)
                                {
                                    i = i - (robot_radius - (j - 1));
                                    break;
                                }

                                // 提前转
                                else if ((floor[i + (j)].y - floor[i + (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    delta = floor[i + (j + 1)].y - floor[i + (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i + (j)].x, floor[i + (j)].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((floor[i + (j + 1)].y - floor[i + (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    path.emplace_back(Point2D(floor[i + (j)].x, floor[i + (j)].y));

                                    delta = floor[i + (j + 1)].y - floor[i + (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i + (j + 1)].x, cell.floor[i + (j + 1)].y - abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(floor[i + (j)]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                }
            }

            if (corner_indicator == BOTTOM_RIGHT)
            {
                int x = 0, y = 0, y_start = 0, y_end = 0;
                bool reverse = false;

                for (int i = ceiling.size() - 1; i >= 0; i = i - (robot_radius + 1))
                {
                    x = ceiling[i].x;

                    if (!reverse)
                    {
                        y_start = floor[i].y;
                        y_end = ceiling[i].y;

                        for (y = y_start; y >= y_end; y--)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(ceiling[i - 1].y - ceiling[i].y) >= 2) && (i - 1 >= 0))
                        {
                            delta = ceiling[i - 1].y - ceiling[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(ceiling[i].x, ceiling[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着ceiling从右往左
                                if (x - j <= ceiling.front().x)
                                {
                                    i = i + (robot_radius - (j - 1));
                                    break;
                                }
                                // 提前转
                                else if ((ceiling[i - (j + 1)].y - ceiling[i - (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i - j].x, ceiling[i - j].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((ceiling[i - (j)].y - ceiling[i - (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    path.emplace_back(ceiling[i - (j)]);

                                    delta = ceiling[i - (j + 1)].y - ceiling[i - (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(ceiling[i - (j + 1)].x, ceiling[i - (j + 1)].y + abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(ceiling[i - j]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                    else
                    {
                        y_start = ceiling[i].y;
                        y_end = floor[i].y;

                        for (y = y_start; y <= y_end; y++)
                        {
                            path.emplace_back(Point2D(x, y));
                        }

                        if ((std::abs(floor[i - 1].y - floor[i].y) >= 2) && (i - 1 >= 0))
                        {
                            delta = floor[i - 1].y - floor[i].y;
                            increment = delta / abs(delta);
                            for (int k = 1; k <= abs(delta); k++)
                            {
                                path.emplace_back(Point2D(floor[i].x, floor[i].y + increment * (k)));
                            }
                        }

                        if (robot_radius != 0)
                        {
                            for (int j = 1; j <= robot_radius + 1; j++)
                            {
                                // 沿着floor从右往左
                                if (x - j <= floor.front().x)
                                {
                                    i = i + (robot_radius - (j - 1));
                                    break;
                                }
                                // 提前转
                                else if ((floor[i - (j)].y - floor[i - (j + 1)].y >= 2) && (j <= robot_radius + 1) && (j + 1 <= robot_radius + 1))
                                {
                                    delta = floor[i - (j + 1)].y - floor[i - (j)].y;
                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i - (j)].x, floor[i - (j)].y + increment * (k)));
                                    }
                                }
                                // 滞后转
                                else if ((floor[i - (j + 1)].y - floor[i - (j)].y >= 2) && (j + 1 <= robot_radius + 1) && (j <= robot_radius + 1))
                                {
                                    path.emplace_back(Point2D(floor[i - (j)].x, floor[i - (j)].y));

                                    delta = floor[i - (j + 1)].y - floor[i - (j)].y;

                                    increment = delta / abs(delta);
                                    for (int k = 0; k <= abs(delta); k++)
                                    {
                                        path.emplace_back(Point2D(floor[i - (j + 1)].x, cell.floor[i - (j + 1)].y - abs(delta) + increment * (k)));
                                    }
                                }
                                else
                                {
                                    path.emplace_back(floor[i - (j)]);
                                }
                            }
                        }

                        reverse = !reverse;
                    }
                }
            }
        }

        return path;
    }

    // 初始化事件列表
    std::vector<Event> CoveragePlanner::initializeEventList(const Polygon &polygon, int polygon_index)
    {
        std::vector<Event> event_list;

        for (const auto &point : polygon)
        {
            event_list.emplace_back(Event(polygon_index, point.x, point.y));
        }

        return event_list;
    }

    void CoveragePlanner::allocateWallEventType(const cv::Mat &map, std::vector<Event> &event_list)
    {
        int index_offset;
        // 只存放各种in和out的index
        std::deque<int> in_out_index_list;

        int N = event_list.size();

        // 检查 in and out and middle
        for (int i = 0; i < N; i++)
        {
            if (event_list[i].x < event_list[((i - 1) % N + N) % N].x && event_list[i].x < event_list[((i + 1) % N + N) % N].x)
            {
                event_list[i].event_type = IN_EX;
                in_out_index_list.emplace_back(i);
            }
            if (event_list[i].x < event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x && event_list[i].y < event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_TOP_EX;
                    in_out_index_list.emplace_back(i);
                }
            }

            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x < event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x && event_list[i].y < event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_TOP_EX;
                    in_out_index_list.emplace_back(i);
                }
            }

            if (event_list[i].x < event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x && event_list[i].y > event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_BOTTOM_EX;
                    in_out_index_list.emplace_back(i);
                }
            }

            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x < event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x && event_list[i].y > event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_BOTTOM_EX;
                    in_out_index_list.emplace_back(i);
                }
            }

            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x)
            {
                event_list[i].event_type = MIDDLE;
            }

            if (event_list[i].x > event_list[((i - 1) % N + N) % N].x && event_list[i].x > event_list[((i + 1) % N + N) % N].x)
            {
                event_list[i].event_type = OUT_EX;
                in_out_index_list.emplace_back(i);
            }

            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x > event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x && event_list[i].y < event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_TOP_EX;
                    in_out_index_list.emplace_back(i);
                }
            }

            if (event_list[i].x > event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x && event_list[i].y < event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_TOP_EX;
                    in_out_index_list.emplace_back(i);
                }
            }

            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x > event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x && event_list[i].y > event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_BOTTOM_EX;
                    in_out_index_list.emplace_back(i);
                }
            }

            if (event_list[i].x > event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x && event_list[i].y > event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_BOTTOM_EX;
                    in_out_index_list.emplace_back(i);
                }
            }
        }

        // determine inner 内点
        Point2D neighbor_point;
        int temp_index;
        for (auto in_out_index : in_out_index_list)
        {
            if (event_list[in_out_index].event_type == OUT_EX)
            {
                neighbor_point = Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255, 255, 255) && neighbor_point.x < map.cols)
                {
                    event_list[in_out_index].event_type = INNER_OUT_EX;
                }
            }

            if (event_list[in_out_index].event_type == OUT_TOP_EX)
            {
                neighbor_point = Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255, 255, 255) && neighbor_point.x < map.cols)
                {
                    event_list[in_out_index].event_type = INNER_OUT_TOP_EX;
                }
            }

            if (event_list[in_out_index].event_type == OUT_BOTTOM_EX)
            {
                neighbor_point = Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255, 255, 255) && neighbor_point.x < map.cols)
                {
                    event_list[in_out_index].event_type = INNER_OUT_BOTTOM_EX;
                }
            }

            if (event_list[in_out_index].event_type == IN_EX)
            {
                neighbor_point = Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255, 255, 255) && neighbor_point.x >= 0)
                {
                    event_list[in_out_index].event_type = INNER_IN_EX;
                }
            }

            if (event_list[in_out_index].event_type == IN_TOP_EX)
            {
                neighbor_point = Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255, 255, 255) && neighbor_point.x >= 0)
                {
                    event_list[in_out_index].event_type = INNER_IN_TOP_EX;
                }
            }

            if (event_list[in_out_index].event_type == IN_BOTTOM_EX)
            {
                neighbor_point = Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(255, 255, 255) && neighbor_point.x >= 0)
                {
                    event_list[in_out_index].event_type = INNER_IN_BOTTOM_EX;
                }
            }
        }

        // determine floor and ceiling 检测顶部和底部
        std::deque<int> ceiling_floor_index_list;

        for (int i = 0; i < in_out_index_list.size(); i++)
        {
            if (
                (event_list[in_out_index_list[0]].event_type == OUT_EX || event_list[in_out_index_list[0]].event_type == OUT_TOP_EX || event_list[in_out_index_list[0]].event_type == OUT_BOTTOM_EX || event_list[in_out_index_list[0]].event_type == INNER_OUT_EX || event_list[in_out_index_list[0]].event_type == INNER_OUT_TOP_EX || event_list[in_out_index_list[0]].event_type == INNER_OUT_BOTTOM_EX) &&
                (event_list[in_out_index_list[1]].event_type == IN_EX || event_list[in_out_index_list[1]].event_type == IN_TOP_EX || event_list[in_out_index_list[1]].event_type == IN_BOTTOM_EX || event_list[in_out_index_list[1]].event_type == INNER_IN_EX || event_list[in_out_index_list[1]].event_type == INNER_IN_TOP_EX || event_list[in_out_index_list[1]].event_type == INNER_IN_BOTTOM_EX))
            {
                if (in_out_index_list[0] < in_out_index_list[1])
                {
                    for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = CEILING;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                }
                else
                {
                    for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = CEILING;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                    for (int k = 0; k < in_out_index_list[1]; k++)
                    {
                        if (event_list[k].event_type != MIDDLE)
                        {
                            event_list[k].event_type = CEILING;
                            ceiling_floor_index_list.emplace_back(k);
                        }
                    }
                }
            }

            if (
                (event_list[in_out_index_list[0]].event_type == IN_EX || event_list[in_out_index_list[0]].event_type == IN_TOP_EX || event_list[in_out_index_list[0]].event_type == IN_BOTTOM_EX || event_list[in_out_index_list[0]].event_type == INNER_IN_EX || event_list[in_out_index_list[0]].event_type == INNER_IN_TOP_EX || event_list[in_out_index_list[0]].event_type == INNER_IN_BOTTOM_EX) &&
                (event_list[in_out_index_list[1]].event_type == OUT_EX || event_list[in_out_index_list[1]].event_type == OUT_TOP_EX || event_list[in_out_index_list[1]].event_type == OUT_BOTTOM_EX || event_list[in_out_index_list[1]].event_type == INNER_OUT_EX || event_list[in_out_index_list[1]].event_type == INNER_OUT_TOP_EX || event_list[in_out_index_list[1]].event_type == INNER_OUT_BOTTOM_EX))
            {
                if (in_out_index_list[0] < in_out_index_list[1])
                {
                    for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = FLOOR;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                }
                else
                {
                    for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = FLOOR;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                    for (int k = 0; k < in_out_index_list[1]; k++)
                    {
                        if (event_list[k].event_type != MIDDLE)
                        {
                            event_list[k].event_type = FLOOR;
                            ceiling_floor_index_list.emplace_back(k);
                        }
                    }
                }
            }

            temp_index = in_out_index_list.front();
            in_out_index_list.pop_front();
            in_out_index_list.emplace_back(temp_index);
        }

        // filter ceiling and floor  过滤顶部和底部
        for (int i = 0; i < ceiling_floor_index_list.size() - 1; i++)
        {
            if (event_list[ceiling_floor_index_list[i]].event_type == CEILING && event_list[ceiling_floor_index_list[i + 1]].event_type == CEILING && event_list[ceiling_floor_index_list[i]].x == event_list[ceiling_floor_index_list[i + 1]].x)
            {
                if (event_list[ceiling_floor_index_list[i]].y > event_list[ceiling_floor_index_list[i + 1]].y)
                {
                    event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
                }
                else
                {
                    event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
                }
            }
            if (event_list[ceiling_floor_index_list[i]].event_type == FLOOR && event_list[ceiling_floor_index_list[i + 1]].event_type == FLOOR && event_list[ceiling_floor_index_list[i]].x == event_list[ceiling_floor_index_list[i + 1]].x)
            {
                if (event_list[ceiling_floor_index_list[i]].y < event_list[ceiling_floor_index_list[i + 1]].y)
                {
                    event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
                }
                else
                {
                    event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
                }
            }
        }
        if (event_list[ceiling_floor_index_list.back()].event_type == CEILING && event_list[ceiling_floor_index_list.front()].event_type == CEILING && event_list[ceiling_floor_index_list.back()].x == event_list[ceiling_floor_index_list.front()].x)
        {
            if (event_list[ceiling_floor_index_list.back()].y > event_list[ceiling_floor_index_list.front()].y)
            {
                event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
            }
        }
        if (event_list[ceiling_floor_index_list.back()].event_type == FLOOR && event_list[ceiling_floor_index_list.front()].event_type == FLOOR && event_list[ceiling_floor_index_list.back()].x == event_list[ceiling_floor_index_list.front()].x)
        {
            if (event_list[ceiling_floor_index_list.back()].y < event_list[ceiling_floor_index_list.front()].y)
            {
                event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
            }
        }
    }

    // 对每个轮廓点做属性分类（特征点）点集默认是逆时针
    void CoveragePlanner::allocateObstacleEventType(const cv::Mat &map, std::vector<Event> &event_list)
    {
        int index_offset;
        // 存放各种in和out的index
        std::deque<int> in_out_index_list;

        int N = event_list.size();
        for (int i = 0; i < N; i++)
        {
            /** // 当前点的x 比 左右两个点的x 都要小
             *    ----- (逆时针)
             *  -          IN
             *    -----
             */
            if (event_list[i].x < event_list[((i - 1) % N + N) % N].x && event_list[i].x < event_list[((i + 1) % N + N) % N].x)
            {
                event_list[i].event_type = IN;
                in_out_index_list.emplace_back(i);
            }

            /** // 当前的 x 比 前面的x要小 且 等于后面的x 且 小于后面的y值，  向后寻找 直到找到x值更大 且 y值更大
             *    --------  （逆时针）
             *   -           IN_TOP
             *   -
             *    --------
             */
            if (event_list[i].x < event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x && event_list[i].y < event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_TOP;
                    in_out_index_list.emplace_back(i);
                }
            }

            /** // 当前的 x 等于的前面x 且小于后面的x 且 小于前面的y值， 向前寻找 直到找到x值更大 且 y值更大
             *    -------  （顺时针）
             *   -          IN_TOP
             *   -
             *    -------
             */
            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x < event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x && event_list[i].y < event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_TOP;
                    in_out_index_list.emplace_back(i);
                }
            }

            /**
             *    -------  （顺时针）
             *   -
             *   -          IN_BOTTOM
             *    -------
             */
            if (event_list[i].x < event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i + index_offset) % N + N) % N].x && event_list[i].y > event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_BOTTOM;
                    in_out_index_list.emplace_back(i);
                }
            }

            /**
             *    -------  （逆时针）
             *   -
             *   -          IN_BOTTOM
             *    -------
             */
            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x < event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x < event_list[((i - index_offset) % N + N) % N].x && event_list[i].y > event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = IN_BOTTOM;
                    in_out_index_list.emplace_back(i);
                }
            }

            /**
             *    -
             *    -    MIDDLE
             *    -
             */
            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x)
            {
                event_list[i].event_type = MIDDLE;
            }

            /**
             *    --------
             *            -  OUT
             *    --------
             */
            if (event_list[i].x > event_list[((i - 1) % N + N) % N].x && event_list[i].x > event_list[((i + 1) % N + N) % N].x)
            {
                event_list[i].event_type = OUT;
                in_out_index_list.emplace_back(i);
            }

            /**
             *   ----------     (逆时针)
             *             -    OUT_TOP
             *             -
             *   ----------
             */
            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x > event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x && event_list[i].y < event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_TOP;
                    in_out_index_list.emplace_back(i);
                }
            }

            /**
             *   ----------     (顺时针)
             *             -    OUT_TOP
             *             -
             *   ----------
             */
            if (event_list[i].x > event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y < event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x && event_list[i].y < event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_TOP;
                    in_out_index_list.emplace_back(i);
                }
            }

            /**
             *   ----------     (顺时针)
             *             -
             *             -    OUT_BOTTOM
             *   ----------
             */
            if (event_list[i].x == event_list[((i - 1) % N + N) % N].x && event_list[i].x > event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i - 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i - index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i - index_offset) % N + N) % N].x && event_list[i].y > event_list[((i - index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_BOTTOM;
                    in_out_index_list.emplace_back(i);
                }
            }
            /**
             *   ----------     (逆时针)
             *             -
             *             -    OUT_BOTTOM
             *   ----------
             */
            if (event_list[i].x > event_list[((i - 1) % N + N) % N].x && event_list[i].x == event_list[((i + 1) % N + N) % N].x && event_list[i].y > event_list[((i + 1) % N + N) % N].y)
            {
                index_offset = 2;
                while (event_list[i].x == event_list[((i + index_offset) % N + N) % N].x)
                {
                    index_offset++;
                }
                if (event_list[i].x > event_list[((i + index_offset) % N + N) % N].x && event_list[i].y > event_list[((i + index_offset) % N + N) % N].y)
                {
                    event_list[i].event_type = OUT_BOTTOM;
                    in_out_index_list.emplace_back(i);
                }
            }
        }

        // determine inner 判断内部点或者是外部点
        Point2D neighbor_point;
        int temp_index;
        for (auto in_out_index : in_out_index_list)
        {

            if (event_list[in_out_index].event_type == OUT)
            {
                neighbor_point = Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0, 0, 0))
                {
                    event_list[in_out_index].event_type = INNER_OUT;
                }
            }

            if (event_list[in_out_index].event_type == OUT_TOP)
            {
                neighbor_point = Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0, 0, 0))
                {
                    event_list[in_out_index].event_type = INNER_OUT_TOP;
                }
            }

            if (event_list[in_out_index].event_type == OUT_BOTTOM)
            {
                neighbor_point = Point2D(event_list[in_out_index].x + 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0, 0, 0))
                {
                    event_list[in_out_index].event_type = INNER_OUT_BOTTOM;
                }
            }

            if (event_list[in_out_index].event_type == IN)
            {
                neighbor_point = Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0, 0, 0))
                {
                    event_list[in_out_index].event_type = INNER_IN;
                }
            }

            if (event_list[in_out_index].event_type == IN_TOP)
            {
                neighbor_point = Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0, 0, 0))
                {
                    event_list[in_out_index].event_type = INNER_IN_TOP;
                }
            }

            if (event_list[in_out_index].event_type == IN_BOTTOM)
            {
                neighbor_point = Point2D(event_list[in_out_index].x - 1, event_list[in_out_index].y);
                if (map.at<cv::Vec3b>(neighbor_point.y, neighbor_point.x) == cv::Vec3b(0, 0, 0))
                {
                    event_list[in_out_index].event_type = INNER_IN_BOTTOM;
                }
            }
        }

        // determine floor and ceiling 判断下面点、上面点
        std::deque<int> ceiling_floor_index_list;
        for (int i = 0; i < in_out_index_list.size(); i++)
        {
            if (
                (event_list[in_out_index_list[0]].event_type == OUT || event_list[in_out_index_list[0]].event_type == OUT_TOP || event_list[in_out_index_list[0]].event_type == OUT_BOTTOM || event_list[in_out_index_list[0]].event_type == INNER_OUT || event_list[in_out_index_list[0]].event_type == INNER_OUT_TOP || event_list[in_out_index_list[0]].event_type == INNER_OUT_BOTTOM) &&
                (event_list[in_out_index_list[1]].event_type == IN || event_list[in_out_index_list[1]].event_type == IN_TOP || event_list[in_out_index_list[1]].event_type == IN_BOTTOM || event_list[in_out_index_list[1]].event_type == INNER_IN || event_list[in_out_index_list[1]].event_type == INNER_IN_TOP || event_list[in_out_index_list[1]].event_type == INNER_IN_BOTTOM))
            {
                if (in_out_index_list[0] < in_out_index_list[1])
                {
                    for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = FLOOR;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                }
                else
                {
                    for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = FLOOR;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                    for (int k = 0; k < in_out_index_list[1]; k++)
                    {
                        if (event_list[k].event_type != MIDDLE)
                        {
                            event_list[k].event_type = FLOOR;
                            ceiling_floor_index_list.emplace_back(k);
                        }
                    }
                }
            }

            if (
                (event_list[in_out_index_list[0]].event_type == IN || event_list[in_out_index_list[0]].event_type == IN_TOP || event_list[in_out_index_list[0]].event_type == IN_BOTTOM || event_list[in_out_index_list[0]].event_type == INNER_IN || event_list[in_out_index_list[0]].event_type == INNER_IN_TOP || event_list[in_out_index_list[0]].event_type == INNER_IN_BOTTOM) &&
                (event_list[in_out_index_list[1]].event_type == OUT || event_list[in_out_index_list[1]].event_type == OUT_TOP || event_list[in_out_index_list[1]].event_type == OUT_BOTTOM || event_list[in_out_index_list[1]].event_type == INNER_OUT || event_list[in_out_index_list[1]].event_type == INNER_OUT_TOP || event_list[in_out_index_list[1]].event_type == INNER_OUT_BOTTOM))
            {
                if (in_out_index_list[0] < in_out_index_list[1])
                {
                    for (int j = in_out_index_list[0] + 1; j < in_out_index_list[1]; j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = CEILING;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                }
                else
                {
                    for (int j = in_out_index_list[0] + 1; j < event_list.size(); j++)
                    {
                        if (event_list[j].event_type != MIDDLE)
                        {
                            event_list[j].event_type = CEILING;
                            ceiling_floor_index_list.emplace_back(j);
                        }
                    }
                    for (int k = 0; k < in_out_index_list[1]; k++)
                    {
                        if (event_list[k].event_type != MIDDLE)
                        {
                            event_list[k].event_type = CEILING;
                            ceiling_floor_index_list.emplace_back(k);
                        }
                    }
                }
            }

            temp_index = in_out_index_list.front();
            in_out_index_list.pop_front();
            in_out_index_list.emplace_back(temp_index);
        }

        // filter ceiling and floor  进一步区分上面点和下面点 滤出中间点
        for (int i = 0; i < ceiling_floor_index_list.size() - 1; i++)
        {
            if (event_list[ceiling_floor_index_list[i]].event_type == CEILING && event_list[ceiling_floor_index_list[i + 1]].event_type == CEILING && event_list[ceiling_floor_index_list[i]].x == event_list[ceiling_floor_index_list[i + 1]].x)
            {
                if (event_list[ceiling_floor_index_list[i]].y > event_list[ceiling_floor_index_list[i + 1]].y)
                {
                    event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
                }
                else
                {
                    event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
                }
            }
            if (event_list[ceiling_floor_index_list[i]].event_type == FLOOR && event_list[ceiling_floor_index_list[i + 1]].event_type == FLOOR && event_list[ceiling_floor_index_list[i]].x == event_list[ceiling_floor_index_list[i + 1]].x)
            {
                if (event_list[ceiling_floor_index_list[i]].y < event_list[ceiling_floor_index_list[i + 1]].y)
                {
                    event_list[ceiling_floor_index_list[i + 1]].event_type = MIDDLE;
                }
                else
                {
                    event_list[ceiling_floor_index_list[i]].event_type = MIDDLE;
                }
            }
        }
        if (event_list[ceiling_floor_index_list.back()].event_type == CEILING && event_list[ceiling_floor_index_list.front()].event_type == CEILING && event_list[ceiling_floor_index_list.back()].x == event_list[ceiling_floor_index_list.front()].x)
        {
            if (event_list[ceiling_floor_index_list.back()].y > event_list[ceiling_floor_index_list.front()].y)
            {
                event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
            }
        }
        if (event_list[ceiling_floor_index_list.back()].event_type == FLOOR && event_list[ceiling_floor_index_list.front()].event_type == FLOOR && event_list[ceiling_floor_index_list.back()].x == event_list[ceiling_floor_index_list.front()].x)
        {
            if (event_list[ceiling_floor_index_list.back()].y < event_list[ceiling_floor_index_list.front()].y)
            {
                event_list[ceiling_floor_index_list.front()].event_type = MIDDLE;
            }
            else
            {
                event_list[ceiling_floor_index_list.back()].event_type = MIDDLE;
            }
        }
    }

    // 对障碍物的点集属性进行标记
    std::vector<Event> CoveragePlanner::generateObstacleEventList(const cv::Mat &map, const PolygonList &polygons)
    {
        std::vector<Event> event_list;
        std::vector<Event> event_sublist;
        for (int i = 0; i < polygons.size(); i++)
        {
            event_sublist = initializeEventList(polygons[i], i);
            allocateObstacleEventType(map, event_sublist);
            event_list.insert(event_list.end(), event_sublist.begin(), event_sublist.end());
            event_sublist.clear();
        }
        std::sort(event_list.begin(), event_list.end());
        return event_list;
    }

    // 对墙体的点集属性进行标记
    std::vector<Event> CoveragePlanner::generateWallEventList(const cv::Mat &map, const Polygon &external_contour)
    {
        std::vector<Event> event_list;
        event_list = initializeEventList(external_contour, INT_MAX);
        allocateWallEventType(map, event_list);
        std::sort(event_list.begin(), event_list.end());
        return event_list;
    }

    // 简单根据x 的值进行分组
    std::deque<std::deque<Event>> CoveragePlanner::sliceListGenerator(const std::vector<Event> &wall_event_list, const std::vector<Event> &obstacle_event_list)
    {
        std::vector<Event> event_list;
        event_list.insert(event_list.end(), obstacle_event_list.begin(), obstacle_event_list.end());
        event_list.insert(event_list.end(), wall_event_list.begin(), wall_event_list.end());
        std::sort(event_list.begin(), event_list.end());
        std::cout << "event_list.size():" << event_list.size() << std::endl;

        std::deque<std::deque<Event>> slice_list;
        std::deque<Event> slice;
        // 最小的x
        int x = event_list.front().x;
        // 找出相同的x,进行粗暴的分组
        for (auto event : event_list)
        {
            if (event.x != x)
            {
                slice_list.emplace_back(slice);
                x = event.x;
                slice.clear();
                slice.emplace_back(event);
            }
            else
            {
                slice.emplace_back(event);
            }
        }
        slice_list.emplace_back(slice);
        return slice_list;
    }

    void CoveragePlanner::executeOpenOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D in, Point2D c, Point2D f, bool rewrite)
    {

        CellNode top_cell, bottom_cell;

        top_cell.ceiling.emplace_back(c);
        top_cell.floor.emplace_back(in);

        bottom_cell.ceiling.emplace_back(in);
        bottom_cell.floor.emplace_back(f);

        if (!rewrite)
        {
            int top_cell_index = cell_graph.size();
            int bottom_cell_index = cell_graph.size() + 1;

            top_cell.cellIndex = top_cell_index;
            bottom_cell.cellIndex = bottom_cell_index;
            cell_graph.emplace_back(top_cell);
            cell_graph.emplace_back(bottom_cell);

            cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
            cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

            cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
            cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
        }
        else
        {
            cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end());
            cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

            int bottom_cell_index = cell_graph.size();
            bottom_cell.cellIndex = bottom_cell_index;
            cell_graph.emplace_back(bottom_cell);

            cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
            cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());
        }
    }

    void CoveragePlanner::executeOpenOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D in_top, Point2D in_bottom, Point2D c, Point2D f, bool rewrite)
    {

        CellNode top_cell, bottom_cell;

        top_cell.ceiling.emplace_back(c);
        top_cell.floor.emplace_back(in_top);

        bottom_cell.ceiling.emplace_back(in_bottom);
        bottom_cell.floor.emplace_back(f);

        if (!rewrite)
        {
            int top_cell_index = cell_graph.size();
            int bottom_cell_index = cell_graph.size() + 1;

            top_cell.cellIndex = top_cell_index;
            bottom_cell.cellIndex = bottom_cell_index;
            cell_graph.emplace_back(top_cell);
            cell_graph.emplace_back(bottom_cell);

            cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
            cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

            cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
            cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
        }
        else
        {
            cell_graph[curr_cell_idx].ceiling.assign(top_cell.ceiling.begin(), top_cell.ceiling.end());
            cell_graph[curr_cell_idx].floor.assign(top_cell.floor.begin(), top_cell.floor.end());

            int bottom_cell_index = cell_graph.size();
            bottom_cell.cellIndex = bottom_cell_index;
            cell_graph.emplace_back(bottom_cell);

            cell_graph[cell_graph[curr_cell_idx].neighbor_indices.back()].neighbor_indices.emplace_back(bottom_cell_index);
            cell_graph[bottom_cell_index].neighbor_indices.emplace_back(cell_graph[curr_cell_idx].neighbor_indices.back());
        }
    }

    void CoveragePlanner::executeFloorOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, const Point2D &floor_point)
    {
        cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
    }

    void CoveragePlanner::executeCeilOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, const Point2D &ceil_point)
    {
        cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
    }

    int CoveragePlanner::countCells(const std::deque<Event> &slice, int curr_idx)
    {
        int cell_num = 0;
        for (int i = 0; i < curr_idx; i++)
        {
            if (
                (slice[i].event_type == IN) || (slice[i].event_type == IN_TOP) || (slice[i].event_type == INNER_IN) || (slice[i].event_type == INNER_IN_BOTTOM) || (slice[i].event_type == FLOOR) || (slice[i].event_type == IN_BOTTOM_EX) || (slice[i].event_type == INNER_IN_EX) || (slice[i].event_type == INNER_IN_TOP_EX))
            {
                cell_num++;
            }
        }
        return cell_num;
    }

    void CoveragePlanner::executeInnerCloseOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D inner_out)
    {
        cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out);
        cell_graph[curr_cell_idx].floor.emplace_back(inner_out);
    }

    void CoveragePlanner::executeInnerCloseOperation(std::vector<CellNode> &cell_graph, int curr_cell_idx, Point2D inner_out_top, Point2D inner_out_bottom)
    {
        cell_graph[curr_cell_idx].ceiling.emplace_back(inner_out_top);
        cell_graph[curr_cell_idx].floor.emplace_back(inner_out_bottom);
    }

    void CoveragePlanner::executeInnerOpenOperation(std::vector<CellNode> &cell_graph, Point2D inner_in)
    {
        CellNode new_cell;
        new_cell.ceiling.emplace_back(inner_in);
        new_cell.floor.emplace_back(inner_in);
        new_cell.cellIndex = cell_graph.size();
        cell_graph.emplace_back(new_cell);
    }

    void CoveragePlanner::executeInnerOpenOperation(std::vector<CellNode> &cell_graph, Point2D inner_in_top, Point2D inner_in_bottom)
    {
        CellNode new_cell;

        new_cell.ceiling.emplace_back(inner_in_top);
        new_cell.floor.emplace_back(inner_in_bottom);

        int new_cell_index = cell_graph.size();

        new_cell.cellIndex = new_cell_index;
        cell_graph.emplace_back(new_cell);
    }

    void CoveragePlanner::executeCloseOperation(std::vector<CellNode> &cell_graph, int top_cell_idx, int bottom_cell_idx, Point2D c, Point2D f, bool rewrite)
    {
        CellNode new_cell;

        new_cell.ceiling.emplace_back(c);
        new_cell.floor.emplace_back(f);

        if (!rewrite)
        {
            int new_cell_idx = cell_graph.size();
            new_cell.cellIndex = new_cell_idx;

            cell_graph.emplace_back(new_cell);

            cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
            cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

            cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
            cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);
        }
        else
        {
            cell_graph[top_cell_idx].ceiling.assign(new_cell.ceiling.begin(), new_cell.ceiling.end());
            cell_graph[top_cell_idx].floor.assign(new_cell.floor.begin(), new_cell.floor.end());

            cell_graph[top_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);
            cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
        }
    }

    std::deque<Event> CoveragePlanner::filterSlice(const std::deque<Event> &slice)
    {
        std::deque<Event> filtered_slice;
        for (auto event : slice)
        {
            // 滤除中间和处于初始状态的
            if (event.event_type != MIDDLE && event.event_type != UNALLOCATED)
            {
                filtered_slice.emplace_back(event);
            }
        }
        return filtered_slice;
    }

    // 区域分解
    void CoveragePlanner::executeCellDecomposition(std::vector<CellNode> &cell_graph, const std::deque<std::deque<Event>> &slice_list)
    {
        std::vector<int> cell_index_slice;
        std::vector<int> original_cell_index_slice;
        int curr_cell_idx = INT_MAX;
        int top_cell_idx = INT_MAX;
        int bottom_cell_idx = INT_MAX;

        Point2D c, f;
        int c_index = INT_MAX;
        int f_index = INT_MAX;
        int min_dist = INT_MAX;

        int event_y = INT_MAX;

        bool rewrite = false;

        std::vector<int> sub_cell_index_slices;
        std::deque<Event> curr_slice;

        int cell_counter = 0;

        for (const auto &raw_slice : slice_list)
        {
            curr_slice = filterSlice(raw_slice);

            original_cell_index_slice.assign(cell_index_slice.begin(), cell_index_slice.end());

            for (int j = 0; j < curr_slice.size(); j++)
            {
                if (curr_slice[j].event_type == INNER_IN_EX)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k]) == original_cell_index_slice.end(); // 若为true，则覆盖

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            curr_cell_idx = cell_index_slice[k];
                            executeOpenOperation(cell_graph, curr_cell_idx,
                                                 Point2D(curr_slice[j].x, curr_slice[j].y),
                                                 c,
                                                 f,
                                                 rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                                sub_cell_index_slices.clear();
                                sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                                cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                            }
                            else
                            {
                                cell_index_slice.insert(cell_index_slice.begin() + k + 1, int(cell_graph.size() - 1));
                            }

                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }
                if (curr_slice[j].event_type == INNER_OUT_EX)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k - 1]) == original_cell_index_slice.end();

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            top_cell_idx = cell_index_slice[k - 1];
                            bottom_cell_idx = cell_index_slice[k];

                            executeCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                  c,
                                                  f,
                                                  rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                            }
                            else
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                            }

                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == INNER_IN_BOTTOM_EX)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k]) == original_cell_index_slice.end();

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            curr_cell_idx = cell_index_slice[k];
                            executeOpenOperation(cell_graph, curr_cell_idx,
                                                 Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), // in top
                                                 Point2D(curr_slice[j].x, curr_slice[j].y),         // in bottom
                                                 c,
                                                 f,
                                                 rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                                sub_cell_index_slices.clear();
                                sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                                cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),
                                                        sub_cell_index_slices.end());
                            }
                            else
                            {
                                cell_index_slice.insert(cell_index_slice.begin() + k + 1, int(cell_graph.size() - 1));
                            }

                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == INNER_OUT_BOTTOM_EX)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k - 1]) == original_cell_index_slice.end();

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            top_cell_idx = cell_index_slice[k - 1];
                            bottom_cell_idx = cell_index_slice[k];
                            executeCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                  c,
                                                  f,
                                                  rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                            }
                            else
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                            }

                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == IN_EX)
                {
                    event_y = curr_slice[j].y;

                    if (!cell_index_slice.empty())
                    {
                        for (int k = 1; k < cell_index_slice.size(); k++)
                        {
                            if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                            {
                                executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_in
                                cell_index_slice.insert(cell_index_slice.begin() + k, int(cell_graph.size() - 1));
                                curr_slice[j].isUsed = true;
                                break;
                            }
                        }
                        if (event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                        {
                            executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_in
                            cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size() - 1));
                            curr_slice[j].isUsed = true;
                        }
                        if (event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                        {
                            executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_in
                            cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size() - 1));
                            curr_slice[j].isUsed = true;
                        }
                    }
                    else
                    {
                        executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_in
                        cell_index_slice.emplace_back(int(cell_graph.size() - 1));
                        curr_slice[j].isUsed = true;
                    }
                }

                if (curr_slice[j].event_type == IN_BOTTOM_EX)
                {
                    event_y = curr_slice[j].y;

                    if (!cell_index_slice.empty())
                    {
                        for (int k = 1; k < cell_index_slice.size(); k++)
                        {
                            if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                            {

                                executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), // inner_in_top,
                                                          Point2D(curr_slice[j].x, curr_slice[j].y));                    // inner_in_bottom

                                cell_index_slice.insert(cell_index_slice.begin() + k, int(cell_graph.size() - 1));

                                curr_slice[j - 1].isUsed = true;
                                curr_slice[j].isUsed = true;

                                break;
                            }
                        }
                        if (event_y <= cell_graph[cell_index_slice.front()].ceiling.back().y)
                        {

                            executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), // inner_in_top,
                                                      Point2D(curr_slice[j].x, curr_slice[j].y));                    // inner_in_bottom

                            cell_index_slice.insert(cell_index_slice.begin(), int(cell_graph.size() - 1));

                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;
                        }
                        if (event_y >= cell_graph[cell_index_slice.back()].floor.back().y)
                        {

                            executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), // inner_in_top,
                                                      Point2D(curr_slice[j].x, curr_slice[j].y));                    // inner_in_bottom

                            cell_index_slice.insert(cell_index_slice.end(), int(cell_graph.size() - 1));

                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;
                        }
                    }
                    else
                    {
                        executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), // inner_in_top,
                                                  Point2D(curr_slice[j].x, curr_slice[j].y));                    // inner_in_bottom

                        cell_index_slice.emplace_back(int(cell_graph.size() - 1));

                        curr_slice[j - 1].isUsed = true;
                        curr_slice[j].isUsed = true;
                    }
                }

                if (curr_slice[j].event_type == OUT_EX)
                {
                    event_y = curr_slice[j].y;

                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            curr_cell_idx = cell_index_slice[k];
                            executeInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_out
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == OUT_BOTTOM_EX)
                {
                    event_y = curr_slice[j].y;

                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            curr_cell_idx = cell_index_slice[k];
                            executeInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_out_top, inner_out_bottom
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                }
            }

            for (int j = 0; j < curr_slice.size(); j++)
            {
                if (curr_slice[j].event_type == IN)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k]) == original_cell_index_slice.end(); // 若为true，则覆盖

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            curr_cell_idx = cell_index_slice[k];
                            executeOpenOperation(cell_graph, curr_cell_idx,
                                                 Point2D(curr_slice[j].x, curr_slice[j].y),
                                                 c,
                                                 f,
                                                 rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                                sub_cell_index_slices.clear();
                                sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                                cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());
                            }
                            else
                            {
                                cell_index_slice.insert(cell_index_slice.begin() + k + 1, int(cell_graph.size() - 1));
                            }

                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }
                if (curr_slice[j].event_type == OUT)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k - 1]) == original_cell_index_slice.end();

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            top_cell_idx = cell_index_slice[k - 1];
                            bottom_cell_idx = cell_index_slice[k];

                            executeCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                  c,
                                                  f,
                                                  rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                            }
                            else
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                            }

                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == IN_BOTTOM)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k]) == original_cell_index_slice.end();

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            curr_cell_idx = cell_index_slice[k];
                            executeOpenOperation(cell_graph, curr_cell_idx,
                                                 Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), // in top
                                                 Point2D(curr_slice[j].x, curr_slice[j].y),         // in bottom
                                                 c,
                                                 f,
                                                 rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                                sub_cell_index_slices.clear();
                                sub_cell_index_slices = {int(cell_graph.size() - 2), int(cell_graph.size() - 1)};
                                cell_index_slice.insert(cell_index_slice.begin() + k, sub_cell_index_slices.begin(),
                                                        sub_cell_index_slices.end());
                            }
                            else
                            {
                                cell_index_slice.insert(cell_index_slice.begin() + k + 1, int(cell_graph.size() - 1));
                            }

                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == OUT_BOTTOM)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if (event_y > cell_graph[cell_index_slice[k - 1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            rewrite = std::find(original_cell_index_slice.begin(), original_cell_index_slice.end(), cell_index_slice[k - 1]) == original_cell_index_slice.end();

                            min_dist = INT_MAX;
                            for (int m = 0; m < curr_slice.size(); m++)
                            {
                                if (abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[m].y - cell_graph[cell_index_slice[k - 1]].ceiling.back().y);
                                    c_index = m;
                                    c = Point2D(curr_slice[m].x, curr_slice[m].y);
                                }
                            }
                            curr_slice[c_index].isUsed = true;

                            min_dist = INT_MAX;
                            for (int n = 0; n < curr_slice.size(); n++)
                            {
                                if (abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y) < min_dist)
                                {
                                    min_dist = abs(curr_slice[n].y - cell_graph[cell_index_slice[k]].floor.back().y);
                                    f_index = n;
                                    f = Point2D(curr_slice[n].x, curr_slice[n].y);
                                }
                            }
                            curr_slice[f_index].isUsed = true;

                            top_cell_idx = cell_index_slice[k - 1];
                            bottom_cell_idx = cell_index_slice[k];
                            executeCloseOperation(cell_graph, top_cell_idx, bottom_cell_idx,
                                                  c,
                                                  f,
                                                  rewrite);

                            if (!rewrite)
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.erase(cell_index_slice.begin() + k - 1);
                                cell_index_slice.insert(cell_index_slice.begin() + k - 1, int(cell_graph.size() - 1));
                            }
                            else
                            {
                                cell_index_slice.erase(cell_index_slice.begin() + k);
                            }

                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == INNER_IN)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {
                            executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_in
                            cell_index_slice.insert(cell_index_slice.begin() + k, int(cell_graph.size() - 1));
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == INNER_IN_BOTTOM)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 1; k < cell_index_slice.size(); k++)
                    {
                        if (event_y >= cell_graph[cell_index_slice[k - 1]].floor.back().y && event_y <= cell_graph[cell_index_slice[k]].ceiling.back().y)
                        {

                            executeInnerOpenOperation(cell_graph, Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), // inner_in_top,
                                                      Point2D(curr_slice[j].x, curr_slice[j].y));                    // inner_in_bottom

                            cell_index_slice.insert(cell_index_slice.begin() + k, int(cell_graph.size() - 1));

                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;

                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == INNER_OUT)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            curr_cell_idx = cell_index_slice[k];
                            executeInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_out
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                }

                if (curr_slice[j].event_type == INNER_OUT_BOTTOM)
                {
                    event_y = curr_slice[j].y;
                    for (int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if (event_y >= cell_graph[cell_index_slice[k]].ceiling.back().y && event_y <= cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            curr_cell_idx = cell_index_slice[k];
                            executeInnerCloseOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j - 1].x, curr_slice[j - 1].y), Point2D(curr_slice[j].x, curr_slice[j].y)); // inner_out_top, inner_out_bottom
                            cell_index_slice.erase(cell_index_slice.begin() + k);
                            curr_slice[j - 1].isUsed = true;
                            curr_slice[j].isUsed = true;
                            break;
                        }
                    }
                }
            }

            for (int j = 0; j < curr_slice.size(); j++)
            {
                if (curr_slice[j].event_type == CEILING)
                {
                    cell_counter = countCells(curr_slice, j);
                    curr_cell_idx = cell_index_slice[cell_counter];
                    if (!curr_slice[j].isUsed)
                    {
                        executeCeilOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                    }
                }
                if (curr_slice[j].event_type == FLOOR)
                {
                    cell_counter = countCells(curr_slice, j);
                    curr_cell_idx = cell_index_slice[cell_counter];
                    if (!curr_slice[j].isUsed)
                    {
                        executeFloorOperation(cell_graph, curr_cell_idx, Point2D(curr_slice[j].x, curr_slice[j].y));
                    }
                }
            }
        }
    }

    /** 测试辅助函数 **/
    void CoveragePlanner::drawObstaclePointType(cv::Mat &map, const Polygon &obstacle)
    {
        PolygonList obstacles = {obstacle};

        std::vector<Event> event_list = generateObstacleEventList(map, obstacles);

        for (auto event : event_list)
        {
            // ----------------------- 绿色 ---------------------------
            {
                if (event.event_type == IN)
                {
                    std::cout << event.x << ", " << event.y << ", IN" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
                }
                if (event.event_type == IN_TOP)
                {
                    std::cout << event.x << ", " << event.y << ", IN_TOP" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
                }
                if (event.event_type == IN_BOTTOM)
                {
                    std::cout << event.x << ", " << event.y << ", IN_BOTTOM" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
                }
                if (event.event_type == INNER_IN)
                {
                    std::cout << event.x << ", " << event.y << ", INNER_IN" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
                }
                if (event.event_type == INNER_IN_TOP)
                {
                    std::cout << event.x << ", " << event.y << ", INNER_IN_TOP" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
                }
                if (event.event_type == INNER_IN_BOTTOM)
                {
                    std::cout << event.x << ", " << event.y << ", INNER_IN_BOTTOM" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
                }
            }
            // --------------------------------------------------

            // ----------------------- 红色 ---------------------------
            {
                if (event.event_type == OUT)
                {
                    std::cout << event.x << ", " << event.y << ", OUT" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
                }
                if (event.event_type == OUT_TOP)
                {
                    std::cout << event.x << ", " << event.y << ", OUT_TOP" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
                }
                if (event.event_type == OUT_BOTTOM)
                {
                    std::cout << event.x << ", " << event.y << ", OUT_BOTTOM" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
                }
                if (event.event_type == INNER_OUT)
                {
                    std::cout << event.x << ", " << event.y << ", INNER_OUT" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
                }
                if (event.event_type == INNER_OUT_TOP)
                {
                    std::cout << event.x << ", " << event.y << ", INNER_OUT_TOP" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
                }
                if (event.event_type == INNER_OUT_BOTTOM)
                {
                    std::cout << event.x << ", " << event.y << ", INNER_OUT_BOTTOM" << std::endl;
                    cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
                }
            }
            // --------------------------------------------------

            // ---------------------- 偏黑色 -----------------------
            if (event.event_type == MIDDLE)
            {
                map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50, 50);
            }
            // ---------------------- 黄色 -----------------------
            if (event.event_type == CEILING)
            {
                map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255, 255);
            }
            // ---------------------- 蓝色 -----------------------
            if (event.event_type == FLOOR)
            {
                map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0, 0);
            }
        }
    }

    void CoveragePlanner::drawWallPointType(cv::Mat &map, const Polygon &wall)
    {
        std::vector<Event> event_list = generateWallEventList(map, wall);
        for (auto event : event_list)
        {
            if (event.event_type == IN_EX)
            {
                std::cout << event.x << ", " << event.y << ", IN_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
            }
            if (event.event_type == IN_TOP_EX)
            {
                std::cout << event.x << ", " << event.y << ", IN_TOP_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
            }
            if (event.event_type == IN_BOTTOM_EX)
            {
                std::cout << event.x << ", " << event.y << ", IN_BOTTOM_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
            }
            if (event.event_type == OUT_EX)
            {
                std::cout << event.x << ", " << event.y << ", OUT_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
            }
            if (event.event_type == OUT_TOP_EX)
            {
                std::cout << event.x << ", " << event.y << ", OUT_TOP_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
            }
            if (event.event_type == OUT_BOTTOM_EX)
            {
                std::cout << event.x << ", " << event.y << ", OUT_BOTTOM_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
            }
            if (event.event_type == INNER_IN_EX)
            {
                std::cout << event.x << ", " << event.y << ", INNER_IN_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
            }
            if (event.event_type == INNER_IN_TOP_EX)
            {
                std::cout << event.x << ", " << event.y << ", INNER_IN_TOP_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
            }
            if (event.event_type == INNER_IN_BOTTOM_EX)
            {
                std::cout << event.x << ", " << event.y << ", INNER_IN_BOTTOM_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 255, 0), -1);
            }
            if (event.event_type == INNER_OUT_EX)
            {
                std::cout << event.x << ", " << event.y << ", INNER_OUT_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
            }
            if (event.event_type == INNER_OUT_TOP_EX)
            {
                std::cout << event.x << ", " << event.y << ", INNER_OUT_TOP_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
            }
            if (event.event_type == INNER_OUT_BOTTOM_EX)
            {
                std::cout << event.x << ", " << event.y << ", INNER_OUT_BOTTOM_EX" << std::endl;
                cv::circle(map, cv::Point(event.x, event.y), 2, cv::Scalar(0, 0, 255), -1);
            }
            if (event.event_type == MIDDLE)
            {
                map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(50, 50, 50);
            }
            if (event.event_type == CEILING)
            {
                map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(0, 255, 255);
            }
            if (event.event_type == FLOOR)
            {
                map.at<cv::Vec3b>(event.y, event.x) = cv::Vec3b(255, 0, 0);
            }
        }
    }

    void CoveragePlanner::printPathNodes(const std::deque<std::deque<Point2D>> &path)
    {
        for (const auto &subpath : path)
        {
            for (const auto &point : subpath)
            {
                std::cout << point.x << ", " << point.y << std::endl;
            }
            std::cout << std::endl;
        }
    }

}