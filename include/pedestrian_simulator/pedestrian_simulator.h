#ifndef __PEDESTRIAN_SIMULATOR_H__
#define __PEDESTRIAN_SIMULATOR_H__

#include <pedestrian_simulator/types.h>
#include <pedestrian_simulator/xml_reader.h>
#include <pedestrian_simulator/pedsim_manager.h>

#include <ros_tools/random_generator.h>
#include <ros_tools/visuals.h>

#include <vector>
#include <Eigen/Dense>

class Pedestrian;
class PedestrianSimulator
{

public:
    PedestrianSimulator();

public:
    RosTools::RandomGenerator random_generator_;

    void ReadScenario();
    void Start();

    void Loop();

    Prediction Publish();

    /** @brief Trajectory predictions for deterministic models (e.g., social forces) */
    void PublishPredictions();
    void PublishGaussianPredictions();

    /** @brief Trajectory predictions for uncertain pedestrian model following a binomial distribution */
    void PublishBinomialPredictions();
    std::vector<Prediction> PublishSocialPredictions();
    void PublishDebugVisuals();
    void VisualizeRobot();
    void VisualizePedestrians();
    void VisualizeStaticObstacles();

private:
    std::unique_ptr<XMLReader> xml_reader_;
    std::unique_ptr<PedsimManager> pedsim_manager_, pedsim_prediction_manager_;
    Eigen::Vector2d vehicle_frame_;
    // geometry_msgs::Pose origin_;

    RobotState robot_state_;

    bool set_N_{false}, set_dt_{false}, set_hz_{false};

    std::vector<std::unique_ptr<Pedestrian>> pedestrians_;

    std::vector<double> colors_ = {217, 83, 25, 0, 114, 189, 119, 172, 48, 126, 47, 142, 237, 177, 32, 77, 190, 238, 162, 19, 47, 256, 153, 256, 0, 103, 256};
    void Reset();
};

#endif // __PEDESTRIAN_SIMULATOR_H__