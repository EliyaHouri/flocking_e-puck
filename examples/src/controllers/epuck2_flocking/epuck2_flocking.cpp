/**
 * @file <epuck2_flocking.cpp>
 *
 * @author Eliyahu Houri
 *
 */

#include "epuck2_flocking.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

/****************************************/
/****************************************/

CEPuck2Flocking::CEPuck2Flocking() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcLedAct(NULL),
    m_pcTOFSensor(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_eState(STATE_START),
    m_strRobotId(""),
    m_uWaitTicks(1) {}  // Initialize wait ticks

/****************************************/
/****************************************/

void SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
    try {
        GetNodeAttribute(t_node, "target_distance", TargetDistance);
        GetNodeAttribute(t_node, "gain", Gain);
        GetNodeAttribute(t_node, "exponent", Exponent);
        GetNodeAttribute(t_node, "max_interaction", MaxInteraction);
        GetNodeAttribute(t_node, "max_wall_interaction", MaxWallInteracation);
        GetNodeAttribute(t_node, "wall_distance", WallDistance);
        GetNodeAttribute(t_node, "goal_distance", GoalDistance);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing flocking parameters.", ex);
    }
}

/****************************************
This function is a generalization of the Lennard-Jones potential
*/
Real SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
    Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
    return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

void CEPuck2Flocking::Init(TConfigurationNode &t_node) {
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_EPuck2ProximitySensor       >("epuck2_proximity"     );
    m_pcLedAct    = GetActuator<CCI_EPuck2LEDsActuator          >("epuck2_leds"          );
    m_pcTOFSensor = GetSensor  <CCI_EPuck2TOFSensor             >("epuck2_tof"           );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );

    m_sFlockingParams.Init(GetNode(t_node, "flocking"));

    //Get the ID of the current robot
    m_strRobotId = GetId();
    
    // Set initial state
    m_eState = STATE_START;
}

/****************************************/
/****************************************/

void CEPuck2Flocking::ControlStep() {
//remove it
    if (m_bIsScheduledForRemoval) {
        // Skip control step if the robot is scheduled for removal
        DisableActuatorsAndSensors();  // Only disable components, but no removal here
        return;
    }

    switch(m_eState) {
        case STATE_START:
            m_eState = STATE_FLOCK;
            m_pcRABAct->SetData(0, STATE_FLOCK);
            break;

        case STATE_FLOCK:
            Flock();
            break;

        default:
            LOGERR << "[BUG] Unknown robot state: " << m_eState << std::endl;
    }
}


/****************************************/
/****************************************

void CEPuck2Flocking::DisableActuatorsAndSensors() {
    // Stop actuators
    m_pcWheels->SetLinearVelocity(0.0, 0.0);
    m_pcLedAct->SetAllBlack();
    m_pcRABAct->ClearData();

    // Disable sensors
    m_pcRABSens->Disable();
    m_pcProximity->Disable();
    m_pcTOFSensor->Disable();
}

/****************************************/
/****************************************/

void CEPuck2Flocking::Flock() {
    CVector2 cDirection = VectorToLight() + FlockingVector() + CalculateWallRepulsionForce();
    m_pcWheels->SetLinearVelocity(cDirection.GetX(), cDirection.GetY());
}

/****************************************/
/****************************************/

CVector2 CEPuck2Flocking::VectorToLight() {
    // Get the robot entity from the space by the robot's ID
    CEPuck2Entity& cEntity = dynamic_cast<CEPuck2Entity&>(CSimulator::GetInstance().GetSpace().GetEntity(GetId()));
    CEmbodiedEntity& cEmbodiedEntity = cEntity.GetEmbodiedEntity();
    const CVector3& cRobotPosition = cEmbodiedEntity.GetOriginAnchor().Position;

    // Get the robot's current heading/orientation
    CRadians cRobotYaw, cRobotPitch, cRobotRoll;
    cEmbodiedEntity.GetOriginAnchor().Orientation.ToEulerAngles(cRobotYaw, cRobotPitch, cRobotRoll);

    // Get the robot's 2D position
    CVector2 cRobotPos2D(cRobotPosition.GetX(), cRobotPosition.GetY());

    // The point we defined as the target ("light")
    CVector2 cLight(0.0, 1.0);

    // Calculate the vector to the light (this gives direction and distance)
    CVector2 cDirectionToLight = cLight - cRobotPos2D;

    // Calculate the angle to the light relative to the robot's current heading
    CRadians cAngleToLight = cDirectionToLight.Angle() - cRobotYaw;

    // Optionally normalize and scale the vector if needed
    if (cDirectionToLight.Length() > 0.01) {
        cDirectionToLight.Normalize();
        cDirectionToLight *= 0.5;  // Apply a small potential force
    }

    // Return the vector pointing toward the light with consideration of angle
    return CVector2(cDirectionToLight.Length(), cAngleToLight);
}

/****************************************/
/****************************************/

CVector2 CEPuck2Flocking::FlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tRABReadings = m_pcRABSens->GetReadings();
    CVector2 cAccum;
    if(!tRABReadings.empty()) {
        for(size_t i = 0; i < tRABReadings.size(); ++i) {
            // Add a check to ensure the reading is valid
            if(tRABReadings[i].Range > 0.0f) {
                Real fLJ = m_sFlockingParams.GeneralizedLennardJones(tRABReadings[i].Range);
                cAccum += CVector2(fLJ, tRABReadings[i].HorizontalBearing);
            }
        }
        if(cAccum.Length() > m_sFlockingParams.MaxInteraction) {
            cAccum.Normalize();
            cAccum *= m_sFlockingParams.MaxInteraction;
        }
    }
    return cAccum;
}


/****************************************/
/****************************************/

bool CEPuck2Flocking::IsCloseToLight() {
    // Get the robot entity from the space by the robot's ID
    CEPuck2Entity& cEntity = dynamic_cast<CEPuck2Entity&>(CSimulator::GetInstance().GetSpace().GetEntity(GetId()));
    CEmbodiedEntity& cEmbodiedEntity = cEntity.GetEmbodiedEntity();
    const CVector3& cRobotPosition = cEmbodiedEntity.GetOriginAnchor().Position;

    // Get the robot's 2D position
    CVector2 cRobotPos2D(cRobotPosition.GetX(), cRobotPosition.GetY());

    // The point we defined as the target ("light")
    CVector2 cLight(0.0, 1.0);

    // Check if the robot is within the goal distance from the light
    return (cRobotPos2D - cLight).Length() < m_sFlockingParams.GoalDistance;
}

/****************************************/
/****************************************/

CVector2 CEPuck2Flocking::CalculateWallRepulsionForce() {
    const CCI_EPuck2ProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    CVector2 cRepulsionVector;

    if(!tProxReads.empty()) {
        for(size_t i = 0; i < tProxReads.size(); ++i) {
            if (tProxReads[i].Value > 0.0f) { //prevent inf's
                // Calculate the magnitude of the repulsion force (inverse relationship)
                Real fRepulsionMagnitude = m_sFlockingParams.GeneralizedLennardJones(tProxReads[i].Value);
                
                // Accumulate the vector
                cRepulsionVector += CVector2((1 / (fRepulsionMagnitude + 10)), tProxReads[i].Angle);
            }
        }
        
        // Check if the vector length exceeds the maximum allowed interaction
        if (cRepulsionVector.Length() > 0.0f) {
            if(cRepulsionVector.Length() > m_sFlockingParams.MaxInteraction) {
                cRepulsionVector.Normalize();
                cRepulsionVector *= m_sFlockingParams.MaxInteraction;
            }
        }
    }

    // Reverse the vector direction
    return cRepulsionVector * -15;
}

/****************************************/
/****************************************/

void CEPuck2Flocking::Reset() {
    m_eState = STATE_START;
}

/****************************************/
/****************************************/

void CEPuck2Flocking::Destroy() {
    // Disable the actuators and sensors
    if(m_pcWheels) {
        m_pcWheels->SetLinearVelocity(0.0, 0.0);
    }
    if(m_pcLedAct) {
        m_pcLedAct->SetAllBlack();  // Turn off all LEDs
    }
    if(m_pcRABAct) {
        m_pcRABAct->ClearData();  // Clear range and bearing data
    }

    // Disable sensors
    if(m_pcProximity) {
        m_pcProximity->Disable();
    }
    if(m_pcRABSens) {
        m_pcRABSens->Disable();
    }
    if(m_pcTOFSensor) {
        m_pcTOFSensor->Disable();
    }

    // Any additional cleanup steps for your robot's controller or components can go here
    LOG << "Destroyed controller for robot: " << m_strRobotId << std::endl;
}


REGISTER_CONTROLLER(CEPuck2Flocking, "epuck2_flocking_controller")

