#ifndef __AUTOWARE_INTERFACE__
#define __AUTOWARE_INTERFACE__

#include <rclcpp/rclcpp.hpp>
#include <pedestrians/pedestrian.h>

#include <pedestrian_simulator/configuration.h>

#include <ros_tools/helpers.h>
#include <ros_tools/ros2_wrappers.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Regular tools
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

// Tier 4 tool?
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <memory>
#include <random>

// class Pedestrian;
namespace pedestrian_simulator
{
#define AUTOWARE_Z 0.0 // 19.5897274017334
#define OBSTACLE_RADIUS 0.5

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
		AutowareInterface(rclcpp::Node *node) : logger_(rclcpp::get_logger("pedestrian_simulator.autoware_interface")), clock_(node->get_clock())
		{
			detected_pub_ = node->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
				"/perception/object_recognition/detection/objects", 1);

			predicted_pub_ = node->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
				"/perception/object_recognition/virtual_objects", 1);

			// pub_ = node->create_publisher<dummy_perception_publisher::msg::Object>(
			// "/simulation/dummy_perception_publisher/object_info", 1);
			detected_with_features_pub_ = node->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
				"/perception/object_recognition/detection/objects_with_feature", 1);

			initial_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
			goal_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 1);

			// A service client for changing directly to autonomous mode
			start_autonomous_client_ = node->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

			node_ = node;
			// rclcpp::QoS qos{1};
			// qos.transient_local();
			// pointcloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/obstacle_segmentation/pointcloud", qos);
		}

		void Publish(const std::vector<std::unique_ptr<Pedestrian>> &pedestrians)
		{
			if (CONFIG.debug_output_)
				RCLCPP_INFO(logger_, "Publishing pedestrians as autoware message.");

			// PublishObjectWithFeatures(pedestrians);
			PublishDetectedPredictedObjects(pedestrians);
		}

	public:
		sensor_msgs::msg::PointCloud2 GetPointCloud(const Pedestrian &ped)
		{
			// Create a sensor_msgs::PointCloud2 message
			sensor_msgs::msg::PointCloud2 pointcloud_msg;
			pointcloud_msg.header.frame_id = "map"; // Set the frame ID to "map"
			pointcloud_msg.header.stamp = clock_->now();

			// Set the fields for the point cloud
			pointcloud_msg.fields.resize(3);
			pointcloud_msg.fields[0].name = "x";
			pointcloud_msg.fields[0].offset = 0;
			pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
			pointcloud_msg.fields[0].count = 1;
			pointcloud_msg.fields[1].name = "y";
			pointcloud_msg.fields[1].offset = 4;
			pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
			pointcloud_msg.fields[1].count = 1;
			pointcloud_msg.fields[2].name = "z";
			pointcloud_msg.fields[2].offset = 8;
			pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
			pointcloud_msg.fields[2].count = 1;

			pointcloud_msg.height = 1;
			pointcloud_msg.width = 1;

			// Reserve space for one point
			pointcloud_msg.data.resize(3 * sizeof(float));

			// Set the point data
			sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
			sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
			sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");

			// Set the point coordinates
			*iter_x = ped.position_.x;
			*iter_y = ped.position_.y;
			*iter_z = AUTOWARE_Z;
			return pointcloud_msg;
		}

		void PublishObjectsWithFeatures(const std::vector<std::unique_ptr<Pedestrian>> &pedestrians)
		{

			std::mt19937 gen(std::random_device{}());
			std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);

			// All are pedestrians
			autoware_auto_perception_msgs::msg::ObjectClassification classification;
			classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
			classification.probability = 1.;

			tier4_perception_msgs::msg::DetectedObjectsWithFeature output;

			for (auto &ped : pedestrians)
			{
				tier4_perception_msgs::msg::DetectedObjectWithFeature detected_object_with_feature;
				auto &detected_object = detected_object_with_feature.object;
				detected_object.classification.push_back(classification);
				detected_object.existence_probability = 1.;

				detected_object.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;
				detected_object.shape.dimensions.x = 2 * OBSTACLE_RADIUS;
				detected_object.shape.dimensions.y = 2 * OBSTACLE_RADIUS; // Not relevant for cylinder
				detected_object.shape.dimensions.z = 1.8;

				detected_object.kinematics.pose_with_covariance.pose.position.x = ped->position_.x;
				detected_object.kinematics.pose_with_covariance.pose.position.y = ped->position_.y;
				detected_object.kinematics.pose_with_covariance.pose.position.z = AUTOWARE_Z;

				double angle = (std::abs(ped->twist_.linear.x) < 1.0e-6 && std::abs(ped->twist_.linear.y) < 1.0e-6)
								   ? 0.
								   : std::atan2(ped->twist_.linear.x, ped->twist_.linear.y);

				detected_object.kinematics.pose_with_covariance.pose.orientation = RosTools::angleToQuaternion(angle);
				detected_object.kinematics.twist_with_covariance.twist.linear.x = ped->twist_.linear.x;
				detected_object.kinematics.twist_with_covariance.twist.linear.y = ped->twist_.linear.y;

				detected_object.kinematics.pose_with_covariance.covariance[0] = 0.1;
				detected_object.kinematics.pose_with_covariance.covariance[7] = 0.1;
				detected_object.kinematics.pose_with_covariance.covariance[14] = 0.01;
				detected_object.kinematics.pose_with_covariance.covariance[35] = 0.1;

				detected_object.kinematics.has_twist_covariance = false;
				detected_object.kinematics.orientation_availability = 2;

				detected_object_with_feature.feature.cluster = GetPointCloud(*ped);

				detected_object.existence_probability = 1.;

				output.feature_objects.push_back(detected_object_with_feature);
				// pointcloud_pub_->publish(detected_object_with_feature.feature.cluster);
			}

			output.header.frame_id = "map";
			output.header.stamp = rclcpp::Clock().now();
			detected_with_features_pub_->publish(output);
		}

		void PublishDetectedPredictedObjects(
			const std::vector<std::unique_ptr<Pedestrian>> &pedestrians)
		{
			std::mt19937 gen(std::random_device{}());
			std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);

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

				object.shape.dimensions.x = 2 * OBSTACLE_RADIUS;
				object.shape.dimensions.y = 2 * OBSTACLE_RADIUS; // Not relevant for cylinder
				object.shape.dimensions.z = 1.8;
				detected_object.shape = object.shape;

				object.kinematics.initial_pose_with_covariance.pose.position.x = ped->position_.x;
				object.kinematics.initial_pose_with_covariance.pose.position.y = ped->position_.y;
				object.kinematics.initial_pose_with_covariance.pose.position.z = AUTOWARE_Z;
				double angle = (std::abs(ped->twist_.linear.x) < 1.0e-6 && std::abs(ped->twist_.linear.y) < 1.0e-6)
								   ? 0.
								   : std::atan2(ped->twist_.linear.x, ped->twist_.linear.y);

				object.kinematics.initial_pose_with_covariance.pose.orientation = RosTools::angleToQuaternion(angle);

				object.kinematics.initial_twist_with_covariance.twist.linear.x = ped->twist_.linear.x;
				object.kinematics.initial_twist_with_covariance.twist.linear.y = ped->twist_.linear.y;
				object.kinematics.initial_pose_with_covariance.covariance[0] = 0.1;
				object.kinematics.initial_pose_with_covariance.covariance[7] = 0.1;
				object.kinematics.initial_pose_with_covariance.covariance[14] = 0.01;
				object.kinematics.initial_pose_with_covariance.covariance[35] = 0.1;

				detected_object.kinematics.pose_with_covariance = object.kinematics.initial_pose_with_covariance;
				detected_object.kinematics.has_twist = false;
				detected_object.kinematics.has_twist_covariance = false;
				detected_object.kinematics.has_position_covariance = false;
				detected_object.kinematics.orientation_availability = 2;

				std::generate(object.object_id.uuid.begin(), object.object_id.uuid.end(), bit_eng);
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

			// detected_result.header.frame_id = "map";
			// detected_result.header.stamp = clock_->now();
			// detected_pub_->publish(detected_result);

			result.header.frame_id = "map";
			result.header.stamp = clock_->now();
			predicted_pub_->publish(result);
		}

		void PublishVehicleInitialPositionAndGoal()
		{
			// Manually copied start and goal from RViz2
			geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
			initial_pose.header.frame_id = "map";
			initial_pose.header.stamp = clock_->now();
			initial_pose.pose.pose.position.x = CONFIG.initial_x_;
			initial_pose.pose.pose.position.y = CONFIG.initial_y_;
			initial_pose.pose.pose.orientation.z = CONFIG.initial_orientation_z_;
			initial_pose.pose.pose.orientation.w = CONFIG.initial_orientation_w_;
			initial_pose_pub_->publish(initial_pose);

			rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1. * 1e9))); // Wait 1s

			geometry_msgs::msg::PoseStamped goal_pose;
			goal_pose.header.frame_id = "map";
			goal_pose.header.stamp = clock_->now();

			goal_pose.pose.position.x = CONFIG.goal_x_;
			goal_pose.pose.position.y = CONFIG.goal_y_;
			goal_pose.pose.orientation.z = CONFIG.goal_orientation_z_;
			goal_pose.pose.orientation.w = CONFIG.goal_orientation_w_;
			goal_pose_pub_->publish(goal_pose);

			// rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(3. * 1e9))); // Wait 3s
			// THE SERVICE CALL SHOULD HAPPEN LATER
		}

		void EnableAutonomousMode()
		{
			RosTools::callServiceWithoutResponse<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(start_autonomous_client_, logger_);
		}

		/** @note: Publishes the rivz dummy (will also simulate point clouds but only constant velocity behavior) */
		// dummy_perception_publisher::msg::Object ToAutowareMessage(
		// 	const std::vector<std::unique_ptr<Pedestrian>> &pedestrians)
		// {

		// 	std::mt19937 gen(std::random_device{}());
		// 	std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);

		// 	// All are pedestrians
		// 	autoware_auto_perception_msgs::msg::ObjectClassification classification;
		// 	classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
		// 	classification.probability = 1.;

		// 	// autoware_auto_perception_msgs::msg::PredictedObjects result;

		// 	for (auto &ped : pedestrians)
		// 	{
		// 		dummy_perception_publisher::msg::Object object;
		// 		object.shape.type = autoware_auto_perception_msgs::msg::Shape::CYLINDER;

		// 		object.shape.dimensions.x = 2 * 0.4;
		// 		object.shape.dimensions.y = 2 * 0.4; // Not relevant for cylinder
		// 		object.shape.dimensions.z = 1.8;

		// 		object.initial_state.pose_covariance.pose.position.x = ped->position_.x;
		// 		object.initial_state.pose_covariance.pose.position.y = ped->position_.y;
		// 		object.initial_state.pose_covariance.pose.position.z = AUTOWARE_Z;
		// 		double angle = (std::abs(ped->twist_.linear.x) < 1.0e-6 && std::abs(ped->twist_.linear.y) < 1.0e-6)
		// 						   ? 0.
		// 						   : std::atan2(ped->twist_.linear.x, ped->twist_.linear.y);

		// 		object.initial_state.pose_covariance.pose.orientation = RosTools::angleToQuaternion(angle);

		// 		object.initial_state.twist_covariance.twist.linear.x = ped->twist_.linear.x;
		// 		object.initial_state.twist_covariance.twist.linear.y = ped->twist_.linear.y;
		// 		object.initial_state.pose_covariance.covariance[0] = 0.1;
		// 		object.initial_state.pose_covariance.covariance[7] = 0.1;
		// 		object.initial_state.pose_covariance.covariance[14] = 0.01;
		// 		object.initial_state.pose_covariance.covariance[35] = 0.1;

		// 		std::generate(object.id.uuid.begin(), object.id.uuid.end(), bit_eng);
		// 		object.classification = classification;
		// 		object.max_velocity = 1.6;
		// 		object.min_velocity = 1.4;
		// 		published_ = true;

		// 		// object.existence_probability = 1.;

		// 		// For now: Constant Velocity Predictions
		// 		// autoware_auto_perception_msgs::msg::PredictedPath path;
		// 		// geometry_msgs::msg::Pose cur_pose;
		// 		// cur_pose = object.kinematics.initial_pose_with_covariance.pose;
		// 		// for (int k = 0; k < 20; k++)
		// 		// {
		// 		// 	// Constant velocity
		// 		// 	cur_pose.position.x += ped->twist_.linear.x * 0.5;
		// 		// 	cur_pose.position.y += ped->twist_.linear.y * 0.5;
		// 		// 	path.path.push_back(cur_pose);

		// 		// 	// With prediction step
		// 		// 	path.time_step = rclcpp::Duration::from_seconds(0.5);
		// 		// }
		// 		// path.confidence = 1.;
		// 		// object.kinematics.predicted_paths.push_back(path);

		// 		// result.objects.push_back(object);
		// 		object.header.frame_id = "map";
		// 		object.header.stamp = clock_->now();
		// 		return object; // ONLY THE FIRST
		// 	}
		// }

	private:
		rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr predicted_pub_;
		rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr detected_pub_;
		// rclcpp::Publisher<dummy_perception_publisher::msg::Object>::SharedPtr pub_;
		rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr detected_with_features_pub_;
		// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;

		rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr start_autonomous_client_;

		rclcpp::Logger logger_;
		rclcpp::Clock::SharedPtr clock_;
		rclcpp::Node *node_;
	};
}

#endif
