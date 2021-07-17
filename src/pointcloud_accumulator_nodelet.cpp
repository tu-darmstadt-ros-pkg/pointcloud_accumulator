#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pointcloud_accumulator/pointcloud_accumulator.h>

namespace pointcloud_accumulator {

    class PointcloudAccumulatorNodelet : public nodelet::Nodelet {
        void onInit() override {
            ros::NodeHandle& nh = getNodeHandle();
            ros::NodeHandle& pnh = getPrivateNodeHandle();
            pointcloud_accumulator_ = std::make_shared<PointcloudAccumulator>(nh, pnh);
            pointcloud_accumulator_->init();
            double update_rate = pnh.param("update_rate", 1.0);
            timer_ = nh.createTimer(ros::Duration(1.0/update_rate), &PointcloudAccumulatorNodelet::timerCb, this, false);
        }

        void timerCb(const ros::TimerEvent&) {
            pointcloud_accumulator_->publish_pointcloud();
        }

        std::shared_ptr<PointcloudAccumulator> pointcloud_accumulator_;
        ros::Timer timer_;

    };
}

PLUGINLIB_EXPORT_CLASS(pointcloud_accumulator::PointcloudAccumulatorNodelet, nodelet::Nodelet);