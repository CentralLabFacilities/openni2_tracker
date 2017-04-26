/*
 * Copyright (c) 2013, Marcus Liebhardt, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Yujin Robot nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Inspired by the openni_tracker by Tim Field and PrimeSense's NiTE 2.0 - Simple Skeleton Sample
 */

#include "openni2_tracker.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <NiTE.h>

OpenNI2Tracker::OpenNI2Tracker(std::string name) : as_(nh_, name, boost::bind(&OpenNI2Tracker::start, this, _1), false) {
    init();
    as_.start();
}

OpenNI2Tracker::~OpenNI2Tracker() {
    nite::NiTE::shutdown();
}

using std::string;

void OpenNI2Tracker::init() {

    for (int i = 0; i < max_users_; ++i) {
        g_visibleUsers.push_back(false);
        g_skeletonStates.push_back(nite::SKELETON_NONE);
    }

    frame_id_ = "openni_depth_frame";
    if (!nh_.getParam("camera_frame_id", frame_id_)) {
        ROS_FATAL("No camera frame.");
        ros::shutdown();
    }
    if (!nh_.getParam("device_id", device_id_)) {
        device_id_ = "#1";
    }
    device_initialization();
}

void OpenNI2Tracker::device_initialization() {
    boost::mutex::scoped_lock lock(mutex_);
    if (openni::OpenNI::initialize() != openni::STATUS_OK) {
        return;
    }

    openni::Array<openni::DeviceInfo> deviceInfoList;
    openni::OpenNI::enumerateDevices(&deviceInfoList);

    if (device_id_.size() == 0 || device_id_ == "#1") {
        device_id_ = deviceInfoList[0].getUri();
    }

    if (devDevice_.open(device_id_.c_str()) != openni::STATUS_OK) {
        return;
    }
}

void OpenNI2Tracker::start(const openni2_tracker_msgs::NiteTrackerGoalConstPtr &goal) {
    if (!as_.isActive() || as_.isPreemptRequested()) {
        ROS_INFO("Preempted");
        as_.setPreempted();
        return;
    }
    userTrackerFrame_.reset(new nite::UserTrackerFrameRef);
    userTracker_.reset(new nite::UserTracker);
    nite::NiTE::initialize();

    niteRc_ = userTracker_->create(&devDevice_);
    if (niteRc_ != nite::STATUS_OK) {
        return;
    }

    boost::mutex::scoped_lock lock(mutex_);
    if (!devDevice_.isValid())
        return;
    niteRc_ = userTracker_->readFrame(&(*userTrackerFrame_));
    if (niteRc_ != nite::STATUS_OK) {
        return;
    }
    const nite::Array<nite::UserData> &users = userTrackerFrame_->getUsers();
    for (int i = 0; i < users.getSize(); ++i) {
        const nite::UserData &user = users[i];
        updateUserState(user, userTrackerFrame_->getTimestamp());
        if (user.isNew()) {
            userTracker_->startSkeletonTracking(user.getId());
        } else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
        }
    }
    openni2_tracker_msgs::NiteTrackerResult result = OpenNI2Tracker::createResult(users);
    as_.setSucceeded(result);
}

openni2_tracker_msgs::NiteTrackerResult OpenNI2Tracker::createResult(const nite::Array<nite::UserData> &users) {

    openni2_tracker_msgs::NiteTrackerResult result = openni2_tracker_msgs::NiteTrackerResult();
    for (int i = 0; i < users.getSize(); ++i) {
        const nite::UserData &user = users[i];

        if (user.getSkeleton().getState() != nite::SKELETON_TRACKED) {
            continue;
        }
        result.skeletons.push_back(createSkeleton(user));
    }
}

openni2_tracker_msgs::Skeleton OpenNI2Tracker::createSkeleton(nite::UserData const &user) {
    openni2_tracker_msgs::Skeleton skel = openni2_tracker_msgs::Skeleton();
    skel.id = user.getId();

    skel.joints.push_back(createJoint(user, "head", nite::JOINT_HEAD));
    skel.joints.push_back(createJoint(user, "neck", nite::JOINT_NECK));
    skel.joints.push_back(createJoint(user, "torso", nite::JOINT_TORSO));

    skel.joints.push_back(createJoint(user, "left_shoulder", nite::JOINT_LEFT_SHOULDER));
    skel.joints.push_back(createJoint(user, "left_elbow", nite::JOINT_LEFT_ELBOW));
    skel.joints.push_back(createJoint(user, "left_hand", nite::JOINT_LEFT_HAND));

    skel.joints.push_back(createJoint(user, "right_shoulder", nite::JOINT_RIGHT_SHOULDER));
    skel.joints.push_back(createJoint(user, "right_elbow", nite::JOINT_RIGHT_ELBOW));
    skel.joints.push_back(createJoint(user, "right_hand", nite::JOINT_RIGHT_HAND));

    skel.joints.push_back(createJoint(user, "left_hip", nite::JOINT_LEFT_HIP));
    skel.joints.push_back(createJoint(user, "left_knee", nite::JOINT_LEFT_KNEE));
    skel.joints.push_back(createJoint(user, "left_foot", nite::JOINT_LEFT_FOOT));

    skel.joints.push_back(createJoint(user, "right_hip", nite::JOINT_RIGHT_HIP));
    skel.joints.push_back(createJoint(user, "right_knee", nite::JOINT_RIGHT_KNEE));
    skel.joints.push_back(createJoint(user, "right_foot", nite::JOINT_RIGHT_FOOT));


    skel.header.frame_id = frame_id_;
    skel.header.stamp = ros::Time::now();

    return skel;
}

openni2_tracker_msgs::Joint OpenNI2Tracker::createJoint(nite::UserData const &user, std::string jointname, nite::JointType const &jointtype) {
    nite::SkeletonJoint joint_position = user.getSkeleton().getJoint(jointtype);
    double x = joint_position.getPosition().x / 1000.0;
    double y = joint_position.getPosition().z / 1000.0;
    double z = joint_position.getPosition().y / 1000.0;

    double qx = joint_position.getOrientation().x;
    double qy = joint_position.getOrientation().z;
    double qz = -joint_position.getOrientation().y;
    double qw = joint_position.getOrientation().w;

    geometry_msgs::Point point = geometry_msgs::Point();
    point.x = x;
    point.x = x;
    point.y = y;

    geometry_msgs::Quaternion quad = geometry_msgs::Quaternion();
    quad.w = qw;
    quad.x = qx;
    quad.y = qy;
    quad.z = qz;

    geometry_msgs::Pose pose = geometry_msgs::Pose();
    pose.orientation = quad;
    pose.position = point;

    openni2_tracker_msgs::Joint joint = openni2_tracker_msgs::Joint();
    joint.name = jointname;
    joint.pose = pose;
    return joint;
}

void OpenNI2Tracker::updateUserState(const nite::UserData &user, unsigned long long ts) {
    g_visibleUsers[user.getId()] = user.isVisible();

    if (g_skeletonStates[user.getId()] != user.getSkeleton().getState()) {
        switch (g_skeletonStates[user.getId()] = user.getSkeleton().getState()) {
            case nite::SKELETON_NONE: ROS_INFO("Stopped tracking.");
                break;
            case nite::SKELETON_CALIBRATING: ROS_INFO("Calibrating...");
                break;
            case nite::SKELETON_TRACKED: ROS_INFO("Tracking!");
                break;
            case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
            case nite::SKELETON_CALIBRATION_ERROR_HANDS:
            case nite::SKELETON_CALIBRATION_ERROR_LEGS:
            case nite::SKELETON_CALIBRATION_ERROR_HEAD:
            case nite::SKELETON_CALIBRATION_ERROR_TORSO: ROS_INFO("Calibration Failed... :-|");
                break;
        }
    }
}

