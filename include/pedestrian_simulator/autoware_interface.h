#ifndef __AUTOWARE_INTERFACE__
#define __AUTOWARE_INTERFACE__

#include <rclcpp/rclcpp.hpp>
#include <pedestrians/pedestrian.h>

#include <pedestrian_simulator/configuration.h>

#include <ros_tools/helpers.h>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
// #include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <memory>

// class Pedestrian;
namespace pedestrian_simulator
{
	inline unique_identifier_msgs::msg::UUID generateUUIDMsg(const std::string &input) // Hoisted from detection_sensor.cpp
	{
		static auto generate_uuid = boost::uuids::name_generator(boost::uuids::random_generator()());
		const auto uuid = generate_uuid(input);

		unique_identifier_msgs::msg::UUID uuid_msg;
		std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());
		return uuid_msg;
	}
	class AutowareInterface
	{
	public:
		AutowareInterface(rclcpp::Node *node) : logger_(rclcpp::get_logger("pedestrian_simulator.autoware_interface"))
		{
			pub_ = node->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
				"/perception/object_recognition/objects", 1);

			detected_pub_ = node->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
				"/perception/object_recognition/detection/objects", 1);
		}

		void Publish(const std::vector<std::unique_ptr<Pedestrian>> &pedestrians)
		{
			if (CONFIG.debug_output_)
				RCLCPP_INFO(logger_, "Publishing pedestrians as autoware message.");

			pub_->publish(ToAutowareMessage(pedestrians));
		}

	public:
		autoware_auto_perception_msgs::msg::PredictedObjects ToAutowareMessage(
			const std::vector<std::unique_ptr<Pedestrian>> &pedestrians)
		{

			// All are pedestrians
			autoware_auto_perception_msgs::msg::ObjectClassification classification;
			classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
			classification.probability = 1.;

			autoware_auto_perception_msgs::msg::PredictedObjects result;
			autoware_auto_perception_msgs::msg::DetectedObjects detected_result;

			for (auto &ped : pedestrians)
			{
				autoware_auto_perception_msgs::msg::DetectedObject detected_object;
				detected_object.classification.push_back(classification);
				detected_object.existence_probability = 1.;

				autoware_auto_perception_msgs::msg::PredictedObject object;
				object.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;

				object.shape.dimensions.x = 2 * 0.4;
				object.shape.dimensions.y = 2 * 0.4; // Not relevant for cylinder
				object.shape.dimensions.z = 1.8;
				detected_object.shape = object.shape;

				object.kinematics.initial_pose_with_covariance.pose.position.x = ped->position_.x;
				object.kinematics.initial_pose_with_covariance.pose.position.y = ped->position_.y;
				object.kinematics.initial_pose_with_covariance.pose.orientation =
					RosTools::angleToQuaternion(std::atan2(ped->twist_.linear.x, ped->twist_.linear.y));

				object.kinematics.initial_twist_with_covariance.twist.linear.x = ped->twist_.linear.x;
				object.kinematics.initial_twist_with_covariance.twist.linear.y = ped->twist_.linear.y;

				detected_object.kinematics.pose_with_covariance = object.kinematics.initial_pose_with_covariance;
				detected_object.kinematics.has_twist = false;
				detected_object.kinematics.has_twist_covariance = false;
				detected_object.kinematics.has_position_covariance = false;
				detected_object.kinematics.orientation_availability = 2;

				object.object_id = generateUUIDMsg(std::to_string(ped->id_));
				object.classification.push_back(classification);

				object.existence_probability = 1.;

				// For now: Constant Velocity Predictions
				autoware_auto_perception_msgs::msg::PredictedPath path;
				geometry_msgs::msg::Pose cur_pose;
				cur_pose = object.kinematics.initial_pose_with_covariance.pose;
				for (int k = 0; k < CONFIG.horizon_N_; k++)
				{
					// Constant velocity
					cur_pose.position.x += ped->twist_.linear.x * CONFIG.prediction_step_;
					cur_pose.position.y += ped->twist_.linear.y * CONFIG.prediction_step_;
					path.path.push_back(cur_pose);

					// With prediction step
					path.time_step = rclcpp::Duration::from_seconds(CONFIG.prediction_step_);
				}
				path.confidence = 1.;
				object.kinematics.predicted_paths.push_back(path);

				result.objects.push_back(object);
				detected_result.objects.push_back(detected_object);
			}

			detected_result.header.frame_id = "map";
			detected_result.header.stamp = rclcpp::Clock().now();
			detected_pub_->publish(detected_result);

			result.header.frame_id = "map";
			result.header.stamp = rclcpp::Clock().now();
			return result;
		}

	private:
		rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr pub_;
		rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr detected_pub_;
		rclcpp::Logger logger_;
	};
}

#endif
