/**
 * @file <flocking_loop_functions.cpp>
 *
 * @author Eliyahu Houri
 *
 */

#include "flocking_loop_functions.h"
#include "../../controllers/epuck2_flocking/epuck2_flocking.h"
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <fstream>
#include <thread>
#include <chrono>



CFlockingLoopFunctions::CFlockingLoopFunctions() : m_unKilledByEnemy(0), m_unReachedTarget(0) {}

void CFlockingLoopFunctions::Init(TConfigurationNode& t_tree) {
    try {
        m_v.reserve(GetSpace().GetEntitiesByType("e-puck2").size());
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
    }
}

void CFlockingLoopFunctions::PostStep() {
    // Get a reference to the enemy robot
    CEPuck2Entity* pcEnemyBot = dynamic_cast<CEPuck2Entity*>(&GetSpace().GetEntity("enemy_robot"));
    CVector3 cEnemyPos = pcEnemyBot->GetEmbodiedEntity().GetOriginAnchor().Position;

    // Containers to track robots to remove due to enemy collision and target reached
    std::vector<std::string> robotsToRemoveForCollision;
    std::vector<std::string> robotsToRemoveForTarget;

    // Iterate through all e-puck robots
    CSpace::TMapPerType cEpuckBots = GetSpace().GetEntitiesByType("e-puck2");
    for (auto it = cEpuckBots.begin(); it != cEpuckBots.end(); ++it) {
        CEPuck2Entity& cEpuckBot = *any_cast<CEPuck2Entity*>(it->second);

        // Skip the enemy bot itself
        if (cEpuckBot.GetId() == "enemy_robot") continue;

        // Get the position of the current robot
        CVector3 cPos = cEpuckBot.GetEmbodiedEntity().GetOriginAnchor().Position;
        Real fDistanceToEnemy = (cEnemyPos - cPos).Length();
        Real fDistanceToTarget = abs(cPos.GetY() - 1.2);

        // Check if the robot is close to the enemy (collision)
        if (fDistanceToEnemy < 0.1) { // 0.1 distance from enemy
            robotsToRemoveForCollision.push_back(cEpuckBot.GetId());
        }
        
        // Check if the robot is close enough to the target
        if (fDistanceToTarget < 0.2) { // 0.2 distance from target
            robotsToRemoveForTarget.push_back(cEpuckBot.GetId());
        }
    }
    
    // Remove robots that collided with the enemy
    for (const std::string& strRobotId : robotsToRemoveForCollision) {
        RemoveRobot(strRobotId);
        LOG << "Robot " << strRobotId << " removed due to collision with enemy." << std::endl;
        ++m_unKilledByEnemy;  // Increment collision count
    }

    // Remove robots that reached the target position
    for (const std::string& strRobotId : robotsToRemoveForTarget) {
        RemoveRobot(strRobotId);
        LOG << "Robot " << strRobotId << " removed due to reaching the target position." << std::endl;
        ++m_unReachedTarget;  // Increment target reached count
    }
}

/********************
write data to the results file
*********************/

void CFlockingLoopFunctions::PostExperiment() {
    // Output the final counts to a text file
    std::ofstream outFile("results.txt");
    outFile << "killed " << m_unKilledByEnemy << "\n";
    outFile << "reached_target " << m_unReachedTarget << "\n";
    outFile << "ticks " << CSimulator::GetInstance().GetSpace().GetSimulationClock() << "\n"; // Add ticks count
    outFile.close();

    LOG << "Results saved to results.txt" << std::endl;
}


/********************
stop the experiment when all robots are gone
*********************/

bool CFlockingLoopFunctions::IsExperimentFinished(){
    // Check if only the enemy robot is left
    CSpace::TMapPerType cEpuckBots = GetSpace().GetEntitiesByType("e-puck2");
    int robot_count = 0;

    for (auto it = cEpuckBots.begin(); it != cEpuckBots.end(); ++it) {
        CEPuck2Entity& cEpuckBot = *any_cast<CEPuck2Entity*>(it->second);
        if (cEpuckBot.GetId() != "enemy_robot") {
            ++robot_count;
        }
    }

    // If no more robots are left except the enemy, stop the simulation
    if (robot_count == 0) {
        LOG << "All robots removed; stopping simulation." << std::endl;
        return true;
    }
    return false;
}

/***********************
 removing robot from the simulation
************************/

void CFlockingLoopFunctions::RemoveRobot(const std::string& strRobotId) {
    try {
        // Retrieve the entity from the space
        CEntity& cEntity = GetSpace().GetEntity(strRobotId);

        // Attempt to cast to CEPuck2Entity to access the controllable components
        CEPuck2Entity* pcRobotEntity = dynamic_cast<CEPuck2Entity*>(&cEntity);
        if (pcRobotEntity != nullptr) {
            // Retrieve the robot's controller and take its address
            CCI_Controller& cController = pcRobotEntity->GetControllableEntity().GetController();
            CCI_Controller* pcController = &cController; // Take the address

            // Perform a dynamic cast to the custom controller to disable actuators and sensors
            CEPuck2Flocking* pcFlockingController = dynamic_cast<CEPuck2Flocking*>(pcController);
            if (pcFlockingController != nullptr) {
                pcFlockingController->DisableActuatorsAndSensors();
            } else {
                LOGERR << "Failed to cast controller to CEPuck2Flocking for robot: " << strRobotId << std::endl;
            }
        }

        // Cast to an embodied entity and remove it from each physics engine
        CEmbodiedEntity* pcEmbodiedEntity = dynamic_cast<CEmbodiedEntity*>(&cEntity);
        if (pcEmbodiedEntity != nullptr) {
            for (auto* pEngine : GetSpace().GetPhysicsEngines()) {
                pEngine->RemoveEntity(*pcEmbodiedEntity);
            }
        }

        // Use the space operation to fully remove the entity from the simulation
        CallEntityOperation<CSpaceOperationRemoveEntity, CSpace, void>(GetSpace(), cEntity);

    } catch (CARGoSException& ex) {
        LOGERR << "Error removing robot '" << strRobotId << "': " << ex.what() << std::endl;
    }
}

REGISTER_LOOP_FUNCTIONS(CFlockingLoopFunctions, "flocking_loop_functions")


