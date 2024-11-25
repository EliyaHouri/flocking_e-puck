#include "robot_xaxis_controller.h"
#include <argos3/core/utility/logging/argos_log.h>

CRobotXAxisController::CRobotXAxisController() :
    m_pcWheels(nullptr),
    m_pcRABAct(nullptr),
    m_unTicks(0),
    m_bMovingForward(true) {}

void CRobotXAxisController::Init(TConfigurationNode& t_node) {
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_unTicks = 0;
    m_bMovingForward = true;
    GetNodeAttributeOrDefault(t_node, "enemy_speed", EnemySpeed, 5.0);

}

void CRobotXAxisController::ControlStep() {
    // Set a unique identifier in the RAB message (for example, 255)
    if (m_pcRABAct) {
        m_pcRABAct->SetData(0, ENEMY_ID);  // 255 is a unique identifier for enemy
    }

    // Move the robot back and forth along the x-axis
    if (m_unTicks++ % 580 == 0) {
        m_bMovingForward = !m_bMovingForward;
    }
    Real fSpeed = m_bMovingForward ? EnemySpeed : -1 * EnemySpeed;
    m_pcWheels->SetLinearVelocity(fSpeed, fSpeed);
}

REGISTER_CONTROLLER(CRobotXAxisController, "robot_xaxis_controller")

