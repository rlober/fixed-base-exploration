#ifndef EXPLORATION_SEQUENCE_H
#define EXPLORATION_SEQUENCE_H

#include "ocra/control/Trajectory/GaussianProcessTrajectory.h"
#include "yarp/os/all.h"
#include <ocra/control/Model.h>
#include <ocra/control/Controller.h>
#include <ocra-icub/Utilities.h>
#include "ocra/control/TaskManagers/TaskSequence.h"




class ExplorationSequence
{
public:
    ExplorationSequence(std::shared_ptr<ocra::Controller> _ctrl, std::shared_ptr<ocra::Model> _model);
    ~ExplorationSequence();
    void initialize();
    void update();
    void close();

private:
    bool attainedGoal(int segmentIndex);
    Eigen::VectorXd mapVarianceToWeights(Eigen::VectorXd& variance);
    void generateNewWaypoints(int segmentIndex);
    Eigen::VectorXd generateTarget(int segmentIndex);

private:
    std::shared_ptr<ocra::Controller> ctrl;
    std::shared_ptr<ocra::Model> model;

    yarp::os::Network yarp;

    yarp::os::Port l_hand_port;
    yarp::os::Port l_hand_target_port;
    yarp::os::Port r_hand_port;
    yarp::os::Port r_hand_target_port;

    std::shared_ptr<ocra::GaussianProcessTrajectory> leftHandTrajectory;
    std::shared_ptr<ocra::GaussianProcessTrajectory> rightHandTrajectory;

    Eigen::VectorXd currentDesiredPosition_leftHand;
    Eigen::VectorXd currentDesiredPosition_rightHand;

    int lHandIndex, rHandIndex;

    std::shared_ptr<ocra::SegCartesianTaskManager> leftHandTask;
    std::shared_ptr<ocra::SegCartesianTaskManager> rightHandTask;
    std::shared_ptr<ocra::FullPostureTaskManager> fullPostureTask;
    std::shared_ptr<ocra::PartialPostureTaskManager> partialPostureTask;






    double maxVariance;
    double resetTimeLeft, resetTimeRight;

    Eigen::MatrixXd desiredPosVelAcc_leftHand;
    Eigen::MatrixXd desiredPosVelAcc_rightHand;


    Eigen::VectorXd desiredVariance_leftHand;
    Eigen::VectorXd desiredVariance_rightHand;
    Eigen::VectorXd desiredWeights_leftHand;
    Eigen::VectorXd desiredWeights_rightHand;

    bool initTrigger;

    Eigen::Array3d varianceThresh;

};

#endif
