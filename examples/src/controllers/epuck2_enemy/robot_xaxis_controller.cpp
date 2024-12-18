#include "robot_xaxis_controller.h"
#include <argos3/core/utility/logging/argos_log.h>

// Constructor
CRobotXAxisController::CRobotXAxisController() :
    m_pcWheels(nullptr),
    m_pcRABAct(nullptr),
    m_bMovingForward(true) {}

// Initialize enemy parameters
void SEnemyParams::Init(TConfigurationNode& t_node) {
    GetNodeAttributeOrDefault(t_node, "enemy_speed", EnemySpeed, 2.0);
}

// Initialize the controller
void CRobotXAxisController::Init(TConfigurationNode& t_node) {
    // Initialize actuators
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcLedAct = GetActuator<CCI_EPuck2LEDsActuator>("epuck2_leds");

    m_bMovingForward = true;

    // Initialize the enemy parameters
    m_sEnemyParams.Init(GetNode(t_node, "enemy"));

    // Initialize LEDs to RED for enemy robots
    m_pcLedAct->SetAllBlack();
    m_pcLedAct->SetAllRedLeds(true); // Red LEDs ON
    m_pcLedAct->SetRGBLed2Color(CColor::RED);
    m_pcLedAct->SetRGBLed4Color(CColor::RED);
    m_pcLedAct->SetRGBLed6Color(CColor::RED);
    m_pcLedAct->SetRGBLed8Color(CColor::RED);
}

// Main control loop
void CRobotXAxisController::ControlStep() {
    // Set a unique identifier in the RAB message
    if (m_pcRABAct) {
        m_pcRABAct->SetData(0, ENEMY_ID);  // 255 is a unique identifier for enemy
    }

    // Get the robot's current position
    CEPuck2Entity* pcEntity = dynamic_cast<CEPuck2Entity*>(&CSimulator::GetInstance().GetSpace().GetEntity(GetId()));
    if (!pcEntity) {
        LOGERR << "Failed to cast entity to CEPuck2Entity for ID: " << GetId() << std::endl;
        return;
    }
    const CVector3& cPosition = pcEntity->GetEmbodiedEntity().GetOriginAnchor().Position;

    // Check if the robot has reached the boundary of the x-axis
    if (cPosition.GetX() >= 1.45) {
        m_bMovingForward = false;  // Reverse direction
    } else if (cPosition.GetX() <= -1.45) {
        m_bMovingForward = true;  // Reverse direction
    }

    // Move the robot based on the direction
    Real fSpeed = m_bMovingForward ? m_sEnemyParams.EnemySpeed : -m_sEnemyParams.EnemySpeed;
    m_pcWheels->SetLinearVelocity(fSpeed * m_sEnemyParams.EnemySpeed, fSpeed * m_sEnemyParams.EnemySpeed);
}

// Register the controller
REGISTER_CONTROLLER(CRobotXAxisController, "robot_xaxis_controller")
