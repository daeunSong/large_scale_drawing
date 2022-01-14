#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "KDTree.hpp"


#define Stroke std::vector<geometry_msgs::Pose>

class DrawingInput {
  public:
    DrawingInput(const std::string &, const char &, const geometry_msgs::Pose &);
    DrawingInput(const std::string &, const std::string &,
                                  const char &, const geometry_msgs::Pose &, const std::vector<double>);

    std::vector<Stroke> strokes;
    std::vector<std::vector<Stroke>> strokes_by_range;

    // regarding input file
    std::string drawing_file_name;
    char color;
    std::string drawing_file_name_full;

    // regarding drawing size
    std::vector<double> size; // width, height
    double target_size = 0.5; // target height
    double ratio;
    geometry_msgs::Pose init_drawing_pose;

    // regarding range splitting
    double max_range = 0.45;  // max range width
    std::vector<std::array<double, 2>> ranges;
    std::vector<double> diffs;

    // regarding wall file and arbitrary drawing
    std::string wall_file_name;
    std::vector<double> wall_pose; // x, y, z, x, y, z, w
    std::vector<double> wall_size; // width,height
    KDTree kdtree;


    // functions
    void setFileName(const std::string file_name, const char color);
    void setTargetSize (const double target_size);
    void setDrawingSize (const double ratio);
    void setCanvasRange ();
    void splitByRange();
    void readDrawingFile ();
    void removeLongLine ();
    int detectRange(geometry_msgs::Pose p);
    double getDist(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
    void recenterDrawings();
    bool contains(std::vector<int> vec, const int elem);
    std::vector<std::string> split(const std::string input, const char delimiter);

    // wall related
    void setKDTree();
    void readDrawingFileArb();
    std::tuple<double, point_t> getXAndQuaternion(point_t &pt);
    std::tuple<double, point_t> getXAndSurfaceNormal(point_t &pt);
    double getVectorSize(point_t &);
    point_t getCrossProduct(point_t &, point_t &);
    tf::Matrix3x3 getRotationMatrix(point_t &, double);
    void setCanvasRangeArb(const std_msgs::Float64MultiArray &ri_ranges);
    void splitByRangeArb(const std_msgs::Float64MultiArray &ri_ranges);
    std::vector<std::vector<double>> matrixMult(const std::vector<std::vector<double>> &A, const std::vector<std::vector<double>> &B);
    std::vector<std::vector<double>> matrixInv(const std::vector<std::vector<double>> &m);
    void relocateDrawingsArb(geometry_msgs::Pose &ridgeback_pose, int range_index);
};
