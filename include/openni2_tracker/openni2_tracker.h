#ifndef OPENNI2_TRACKER_NODELET_CLASS_H_
#define OPENNI2_TRACKER_NODELET_CLASS_H_

#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <actionlib/server/simple_action_server.h>
#include <openni2_tracker/NiteTrackerAction.h>

#include <NiTE.h>
#include <OpenNI.h>

namespace openni2_tracker {

    class OpenNI2TrackerNodelet : public nodelet::Nodelet {
    public:
        OpenNI2TrackerNodelet(std::string name);

        ~OpenNI2TrackerNodelet();

        void onInit();

        void start(const openni2_tracker::NiteTrackerActionGoalConstPtr &goal);

        openni2_tracker::NiteTrackerResult createResult(const nite::Array<nite::UserData> &users);
        
        openni2_tracker::Skeleton createSkeleton(nite::UserData const &user);
        
        openni2_tracker::Joint createJoint(nite::UserData const &user, std::string jointname, nite::JointType const &jointtype);

        void updateUserState(const nite::UserData &user,
                             unsigned long long ts);

        void device_initialization();

    private:
        ros::NodeHandle nh_;
        boost::shared_ptr<ros::NodeHandle> pnh_;
        boost::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber depth_img_sub_;
        ros::Timer publish_timer_;
        tf::TransformBroadcaster broadcaster_;

        //actionlib
        actionlib::SimpleActionServer<openni2_tracker::NiteTrackerAction> as_;
        bool running_ = false;

        boost::mutex mutex_;
        boost::shared_ptr<nite::UserTrackerFrameRef> userTrackerFrame_;
        boost::shared_ptr<nite::UserTracker> userTracker_;
        nite::Status niteRc_;
        std::vector<bool> g_visibleUsers;
        std::vector<nite::SkeletonState> g_skeletonStates;
        openni::Device devDevice_;

        std::string frame_id_;
        std::string device_id_;
        double publish_period_;
        int max_users_;
    };

} // namespace openni2_tracker

#endif /* OPENNI2_TRACKER_NODELET_CLASS_H_ */
