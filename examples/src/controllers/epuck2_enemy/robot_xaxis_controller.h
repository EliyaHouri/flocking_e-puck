#ifndef ROBOT_XAXIS_CONTROLLER_H
#define ROBOT_XAXIS_CONTROLLER_H

#define ENEMY_ID 255

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

using namespace argos;

// Struct to hold parameters related to the enemy robot
struct SEnemyParams {
    Real EnemySpeed;  // Speed of the enemy robot

    void Init(TConfigurationNode& t_node);
};

class CRobotXAxisController : public CCI_Controller {
public:
    CRobotXAxisController();
    virtual ~CRobotXAxisController() {}

    virtual void Init(TConfigurationNode& t_node) override;
    virtual void ControlStep() override;
    virtual void Reset() override {}
    virtual void Destroy() override {}

private:
    CCI_DifferentialSteeringActuator* m_pcWheels;
    CCI_RangeAndBearingActuator* m_pcRABAct;
    bool m_bMovingForward;

    SEnemyParams m_sEnemyParams;  // Struct to hold enemy-specific parameters
};

#endif
