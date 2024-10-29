/**
 * @file <flocking_loop_functions.cpp>
 *
 * @author Eliyahu Houri
 *
 */

#include "flocking_loop_functions.h"

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>

CFlockingLoopFunctions::CFlockingLoopFunctions() {
}

void CFlockingLoopFunctions::Init(TConfigurationNode& t_tree) {
    try {
        m_v.reserve(GetSpace().GetEntitiesByType("e-puck2").size());
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
    }
}

void CFlockingLoopFunctions::PostStep() {
    m_v.clear();
    // Make a copy of the entities map
    CSpace::TMapPerType cEpuckbotsCopy = GetSpace().GetEntitiesByType("e-puck2");
    std::vector<std::string> robotsToRemove;

    for(CSpace::TMapPerType::iterator it = cEpuckbotsCopy.begin();
        it != cEpuckbotsCopy.end();
        ++it) {

        CEPuck2Entity& cEpuckBot = *any_cast<CEPuck2Entity*>(it->second);
        CVector3 cPos = cEpuckBot.GetEmbodiedEntity().GetOriginAnchor().Position;
        m_v.push_back(CVector2(cPos.GetX(), cPos.GetY()));

        if(IsCloseToTarget(cEpuckBot)) {
            std::string strRobotId = cEpuckBot.GetId();
            robotsToRemove.push_back(strRobotId);
        }
    }

    // Remove robots after checking all entities
    for (const std::string& strRobotId : robotsToRemove) {
        RemoveRobot(strRobotId);
    }
}



void CFlockingLoopFunctions::PostExperiment() {
    // Any final steps after the experiment ends
}

bool CFlockingLoopFunctions::IsCloseToTarget(CEPuck2Entity& cEpuckBot) {
    // Get the robot's position
    const CVector3& cPos = cEpuckBot.GetEmbodiedEntity().GetOriginAnchor().Position;
    CVector2 cRobotPos2D(cPos.GetX(), cPos.GetY());

    // Define the target position, for example, (0,1)
    CVector2 cTarget(0.0, 1.0);

    // Return true if the robot is close enough to the target
    return (cRobotPos2D - cTarget).Length() < 0.2;  // Adjust threshold as needed
}

void CFlockingLoopFunctions::RemoveRobot(const std::string& strRobotId) {
    try {
        // Fetch the robot entity
        CEPuck2Entity& cEntity = dynamic_cast<CEPuck2Entity&>(GetSpace().GetEntity(strRobotId));

        // Disable the controller, sensors, and actuators to ensure cleanup
        cEntity.GetControllableEntity().GetController().Disable();  // Destroy controller
        cEntity.GetControllableEntity().Destroy();  // Disable all actuators/sensors

        // Remove the robot entity safely
        LOG << "Removing robot: " << strRobotId << std::endl;
        GetSpace().RemoveEntity(cEntity);

    } catch (CARGoSException& ex) {
        LOGERR << "Error removing robot '" << strRobotId << "': " << ex.what() << std::endl;
    }
}

REGISTER_LOOP_FUNCTIONS(CFlockingLoopFunctions, "flocking_loop_functions")

