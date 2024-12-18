/**
 * @file <epuck2_flocking.h>
 *
 * @author Eliyahu Houri
 *
 */

#ifndef EPUCK2_FLOCKING_H
#define EPUCK2_FLOCKING_H

#define ENEMY_ID 255

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

using namespace argos;

// Struct to hold parameters related to flocking interactions
struct SFlockingInteractionParams {
    Real TargetDistance;
    Real Gain;
    Real Exponent;
    Real MaxInteraction;
    Real MaxWallInteracation;
    Real WallDistance;
    Real GoalDistance;
    Real RepulsionForce;
    Real Noise;
    Real SpeedFactor;
    Real LightStrength;
    Real FriendRepulsionForce;
    Real EnemyRepulsionForce;

    void Init(TConfigurationNode& t_node);
    Real GeneralizedLennardJones(Real f_distance);
};

class CEPuck2Flocking : public CCI_Controller {

public:

    CEPuck2Flocking();
    virtual ~CEPuck2Flocking() {}

    virtual void Init(TConfigurationNode &t_node);
    virtual void ControlStep();
    virtual void Reset();
    virtual void Destroy();
    void DisableActuatorsAndSensors();
    Real target = 7.0;

private:

    enum EState {
        STATE_START = 0,
        STATE_FLOCK,
        STATE_WAIT,
        STATE_STOP
    };
        UInt32 m_uWaitTicks;  // Counter for wait ticks
        bool m_bIsScheduledForRemoval;  // Add this variable for removal flagging


private:

    void Flock();
    CVector2 VectorToLight();
    CVector2 FlockingVector();
    CVector2 CalculateWallRepulsionForce();
    bool IsCloseToLight();

    CCI_DifferentialSteeringActuator *m_pcWheels;
    CCI_EPuck2ProximitySensor *m_pcProximity;
    CCI_EPuck2LEDsActuator *m_pcLedAct;
    CCI_EPuck2TOFSensor *m_pcTOFSensor;
    CCI_RangeAndBearingActuator *m_pcRABAct;
    CCI_RangeAndBearingSensor *m_pcRABSens;

    SFlockingInteractionParams m_sFlockingParams;
    EState m_eState;
    std::string m_strRobotId;
    UInt32 m_uStraightTicks;  // Ticks to move straight after reaching the center

};

#endif

