#ifndef ROBOT_XAXIS_CONTROLLER_H
#define ROBOT_XAXIS_CONTROLLER_H

#define ENEMY_ID 255
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

using namespace argos;

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
    UInt32 m_unTicks;
    bool m_bMovingForward;
    Real EnemySpeed;
};

#endif

