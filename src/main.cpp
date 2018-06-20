#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <ros/time.h>
#include "NonlinearOptimization.h"

#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(1);

    std::vector<std::pair<float, float>> raw_path;
    raw_path.push_back(std::make_pair(0, 0));
    raw_path.push_back(std::make_pair(10, 0.));
    raw_path.push_back(std::make_pair(20, 0.0));
    raw_path.push_back(std::make_pair(30, 0.0));
    raw_path.push_back(std::make_pair(40, 0));
    raw_path.push_back(std::make_pair(50, 0));
    raw_path.push_back(std::make_pair(60, 3.75));
    raw_path.push_back(std::make_pair(70, 3));
    raw_path.push_back(std::make_pair(80, 3));
    raw_path.push_back(std::make_pair(90, 3));

    std::vector<std::pair<float, float>> final_path;
    final_path.resize(raw_path.size());
    final_path.assign(raw_path.begin(), raw_path.end());

    float alpha = 0.5;
    float beta = 0.5;

    ros::Time prev = ros::Time::now();

    for (int i = 0; i < 8; ++i)
    {
        for (int j = 1; j < raw_path.size() - 1; ++j)
        {
            final_path[j].first = final_path[j].first + alpha * (raw_path[j].first - final_path[j].first) +
                                  beta * (final_path[j - 1].first - 2 * final_path[j].first + final_path[j + 1].first);
            final_path[j].second = final_path[j].second + alpha * (raw_path[j].second - final_path[j].second) +
                                   beta * (final_path[j - 1].second - 2 * final_path[j].second + final_path[j + 1].second);
        }
    }

    std::cout << "time cost: " << (ros::Time::now() - prev).toNSec() << std::endl;

    // obj = A * sin(Bx) + C * cos(D*x) - F
    //there are 4 parameter: A, B, C, D.
    int num_params = 3;

    //generate random data using these parameter
    int total_data = 100;

    VectorXd input(total_data);
    VectorXd output(total_data);

    double A = 1, B = 2, C = 1;
    //load observation data
    for (int i = 0; i < total_data; i++)
    {
        //generate a random variable [-10 10]
        double x = 20.0 * ((random() % 1000) / 1000.0) - 10.0;
        double deltaY = 2.0 * (random() % 1000) / 1000.0 - 1.0;
        double y = A * x * x * x + B * x * x + C * x + 1;
        // double y = A * x * x * x + B * x * x + C * x + 1 + deltaY;

        input(i) = x;
        output(i) = y;
    }

    //gauss the parameters
    VectorXd params_gaussNewton(num_params);
    //init gauss
    params_gaussNewton << 1.6, 1.4, 1.2;

    VectorXd params_levenMar = params_gaussNewton;
    VectorXd params_dogLeg = params_gaussNewton;

    gaussNewton(input, output, params_gaussNewton);
    levenMar(input, output, params_levenMar);
    dogLeg(input, output, params_dogLeg);

    cout << "gauss newton parameter: " << endl
         << params_gaussNewton << endl
         << endl
         << endl;
    cout << "Levenberg-Marquardt parameter: " << endl
         << params_levenMar << endl
         << endl
         << endl;
    cout << "dog-leg parameter: " << endl
         << params_dogLeg << endl
         << endl
         << endl;

    while (ros::ok())
    {

        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base_link";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::POINTS;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        line_strip.scale.x = 0.1;

        line_list.scale.x = 0.2;
        line_list.scale.y = 0.2;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        for (size_t i = 0; i < raw_path.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = raw_path[i].first;
            p.y = raw_path[i].second;
            ;
            p.z = 5;
            points.points.push_back(p);
        }

        for (size_t i = 0; i < final_path.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = final_path[i].first;
            p.y = final_path[i].second;
            ;
            p.z = 5;

            line_strip.points.push_back(p);
            // The line list needs two points for each line
            line_list.points.push_back(p);
        }

        marker_pub.publish(points);
        marker_pub.publish(line_strip);
        marker_pub.publish(line_list);

        r.sleep();
    }
}
