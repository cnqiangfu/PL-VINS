#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 5, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            for (auto &it : _points)
            {
                int feature_id = it.first;
                int camera_id = it.second[0].first;
                double x = it.second[0].second.x();
                double y = it.second[0].second.y();
                double z = it.second[0].second.z();
                points[feature_id].emplace_back(camera_id,  Vector3d(x,y,z));
            }
        };
        ImageFrame(const map<int, vector<pair<int, Vector3d>>>& _points, double _t):points{_points},t{_t},is_key_frame{false}        {

        };
        map<int, vector<pair<int, Vector3d> > > points;
        double t;
        Matrix3d R;
        Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);