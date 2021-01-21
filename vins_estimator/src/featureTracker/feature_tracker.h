/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);
void reduceVector(vector<cv::KeyPoint> &v, vector<uchar> status);
void reduceMat(cv::Mat &origin_m, cv::Mat &new_m, vector<uchar> matche_status);
void convertPoint(vector<cv::KeyPoint> kpt_v, vector<cv::Point2f> &pt_v);
void combineVector(vector<cv::KeyPoint> &kpt_v, vector<cv::KeyPoint> &first_v, vector<cv::KeyPoint> &second_v);
void combineMat(cv::Mat &dcp, cv::Mat &first_dcp, cv::Mat &second_dcp);

class FeatureTracker
{
public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void descriptorsMatch(vector<cv::DMatch> &match_pair, vector<cv::DMatch> &good_match_pair);
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImageORB(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void extractTrackedPoint(vector<cv::DMatch> &match_pair, vector<uchar> &match_status, vector<cv::KeyPoint> &v, vector<cv::KeyPoint> &tracked_v, cv::Mat &m, cv::Mat &tracked_m, set<int> &tracked_idx);
    void extractUntrackedPoint(set<int> &tracked_idx, vector<cv::KeyPoint> &v, vector<cv::KeyPoint> &new_v, cv::Mat &m, cv::Mat &new_m);
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    //void drawTrackORB(const cv::Mat &imPrev, const cv::Mat &imCur, vector<int>)
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    //cv::Mat getTrackImageORB();
    bool inBorder(const cv::Point2f &pt);
    void updataIdx(vector<int> &ids, map<int, int> kpts_id, map<int, int> id_kpts)
    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;
    vector<cv::KeyPoint> prev_keypoints, cur_keypoints, cur_right_keypoints;
    vector<cv::KeyPoint> cur_tracked_keypoints, cur_untracked_keypoints;
    cv::Mat prev_descriptors, cur_descriptors, cur_right_descriptors;
    cv::Mat cur_tracked_descriptors, cur_untracked_descriptors;
    cv::Ptr<cv::ORB> orb;
    //cv::Ptr<cv::FeatureDetector> detector;
    //cv::Ptr<cv::DescriptorExtractor> descriptor;
    //cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Ptr<cv::BFMatcher> matcher;
    vector<cv::DMatch> matches, good_matches;
    cv::Mat imTrackORB;
    set<int> tracked_idx;
    cv::Mat imTrackPrevCur;
    map<int, int> prev_kpts_id_map, prev_id_kpts_map;
    map<int, int> cur_kpts_id_map, cur_id_kpts_map;
    map<int, int> cur_prev_kpt_map;
};