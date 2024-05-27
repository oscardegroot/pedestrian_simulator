#ifndef __PEDESTRIAN_SIMULATOR_H__
#define __PEDESTRIAN_SIMULATOR_H__
#include <pedestrian_simulator/pedsim_manager.h>

#include <pedestrian_simulator/types.h>
#include <pedestrian_simulator/xml_reader.h>

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

    void Loop();
    void Reset();
    void ResetToStart();

    std::vector<Prediction> GetPedestrians();
    std::vector<Prediction> GetPredictions();

    /** @brief Trajectory predictions for deterministic models (e.g., social forces) */
    void PublishPredictions();

    /** @brief Trajectory predictions for uncertain pedestrian model following a binomial distribution */
    // void PublishBinomialPredictions();
    std::vector<Prediction> GetSocialPredictions();
    std::vector<Prediction> GetGaussianPredictions();

    void SetRobotState(const RobotState &state);

    void PublishDebugVisuals();
    void VisualizeRobot();
    void VisualizePedestrians();
    void VisualizeStaticObstacles();

    RobotState robot_state_;

private:
    std::unique_ptr<XMLReader> xml_reader_;
    std::unique_ptr<PedsimManager> pedsim_manager_, pedsim_prediction_manager_;
    // geometry_msgs::Pose origin_;

    std::vector<std::unique_ptr<Pedestrian>> pedestrians_;

    std::vector<double> colors_ = {217, 83, 25, 0, 114, 189, 119, 172, 48, 126, 47, 142, 237, 177, 32, 77, 190, 238, 162, 19, 47, 256, 153, 256, 0, 103, 256};
};

#endif // __PEDESTRIAN_SIMULATOR_H__