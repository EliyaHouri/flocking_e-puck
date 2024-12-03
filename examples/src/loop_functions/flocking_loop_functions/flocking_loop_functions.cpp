/**
 * @file <flocking_loop_functions.cpp>
 *
 * @author Eliyahu Houri
 *
 * ### TO SWITCH BACK TO COLLISION-BASED REMOVAL ###
 * 1. Uncomment the sections marked as "Collision-Based Removal".
 * 2. Comment out the tracking functions and their calls in `PostStep`.
 */

#include "flocking_loop_functions.h"
#include "../../controllers/epuck2_flocking/epuck2_flocking.h"
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <fstream>

CFlockingLoopFunctions::CFlockingLoopFunctions() : 
    m_unKilledByEnemy(0), 
    m_unReachedTarget(0), 
    m_unTotalCollisions(0),
    m_cooldownTicks(20) {} // Cooldown duration in ticks (e.g., 10 ticks = 1 second)

void CFlockingLoopFunctions::Init(TConfigurationNode& t_tree) {
    try {
        m_v.reserve(GetSpace().GetEntitiesByType("e-puck2").size());
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
    }
}

void CFlockingLoopFunctions::TrackDistanceTraveled(const std::string& robot_id, const CVector3& current_position) {
    if (m_mapPreviousPositions.find(robot_id) != m_mapPreviousPositions.end()) {
        Real fDistance = (current_position - m_mapPreviousPositions[robot_id]).Length();
        m_mapDistanceTraveled[robot_id] += fDistance;
    }
    m_mapPreviousPositions[robot_id] = current_position;
}

void CFlockingLoopFunctions::CheckCollisionsWithEnemy(const std::string& robot_id, const CVector3& robot_position, const CVector3& enemy_position) {
    Real fDistanceToEnemy = (robot_position - enemy_position).Length();
    if (fDistanceToEnemy < 0.1) { // Adjust threshold as needed
        UInt32 current_tick = CSimulator::GetInstance().GetSpace().GetSimulationClock();
        if (m_mapLastCollisionTick.find(robot_id) == m_mapLastCollisionTick.end() ||
            current_tick - m_mapLastCollisionTick[robot_id] >= m_cooldownTicks) {
            m_mapCollisionCount[robot_id]++;
            m_unTotalCollisions++;
            m_mapLastCollisionTick[robot_id] = current_tick; // Update last collision tick
        }
    }
}

bool CFlockingLoopFunctions::IsCloseToTarget(CEPuck2Entity& cEpuckBot) {
    const CVector3& cPosition = cEpuckBot.GetEmbodiedEntity().GetOriginAnchor().Position;
    Real fDistanceToTarget = abs(cPosition.GetY() - target); // Adjust Y-coordinate target
    return fDistanceToTarget < 0.2;
}

void CFlockingLoopFunctions::PostStep() {
    // Iterate through all e-puck robots
    CSpace::TMapPerType cEpuckBots = GetSpace().GetEntitiesByType("e-puck2");

    for (auto it = cEpuckBots.begin(); it != cEpuckBots.end(); ++it) {
        CEPuck2Entity& cEpuckBot = *any_cast<CEPuck2Entity*>(it->second);
        std::string strRobotId = cEpuckBot.GetId();

        // Check if the robot is an enemy robot
        if (strRobotId.find("enemy_robot") != std::string::npos) {
            continue; // Skip enemy robots for distance tracking or removal
        }

        // Get the position of the current robot
        CVector3 cCurrentPosition = cEpuckBot.GetEmbodiedEntity().GetOriginAnchor().Position;

        // Track distance traveled (common for both methods)
        TrackDistanceTraveled(strRobotId, cCurrentPosition);

        // *** Collision Tracking Method (Default) ***
        // Uncomment this block if you want to track collisions
        /*
        for (auto enemy_it = cEpuckBots.begin(); enemy_it != cEpuckBots.end(); ++enemy_it) {
            CEPuck2Entity& cEnemyBot = *any_cast<CEPuck2Entity*>(enemy_it->second);
            std::string strEnemyId = cEnemyBot.GetId();

            if (strEnemyId.find("enemy_robot") != std::string::npos) { // Match dynamic enemy IDs
                CVector3 cEnemyPosition = cEnemyBot.GetEmbodiedEntity().GetOriginAnchor().Position;
                CheckCollisionsWithEnemy(strRobotId, cCurrentPosition, cEnemyPosition);
            }
        }
        */

        // *** Robot Removal Method ***
        // Uncomment this block if you want to remove robots on collision
        for (auto enemy_it = cEpuckBots.begin(); enemy_it != cEpuckBots.end(); ++enemy_it) {
            CEPuck2Entity& cEnemyBot = *any_cast<CEPuck2Entity*>(enemy_it->second);
            std::string strEnemyId = cEnemyBot.GetId();

            if (strEnemyId.find("enemy_robot") != std::string::npos) { // Match dynamic enemy IDs
                CVector3 cEnemyPosition = cEnemyBot.GetEmbodiedEntity().GetOriginAnchor().Position;
                Real fDistanceToEnemy = (cEnemyPosition - cCurrentPosition).Length();
                if (fDistanceToEnemy < 0.1) { // Adjust collision threshold as needed
                    RemoveRobot(strRobotId);
                    ++m_unKilledByEnemy;
                    LOG << "Robot " << strRobotId << " removed due to collision with enemy " << strEnemyId << "." << std::endl;
                    break; // Exit the enemy loop since the robot is already removed
                }
            }
        }

        // Check if the robot reached the target
        if (IsCloseToTarget(cEpuckBot)) {
            RemoveRobot(strRobotId);
            ++m_unReachedTarget;
            LOG << "Robot " << strRobotId << " reached the target and was removed." << std::endl;
        }
    }
}


void CFlockingLoopFunctions::PostExperiment() {
    std::ofstream outFile("results.txt");

    // Sort robot IDs for output
    std::vector<std::string> sorted_robot_ids;
    for (const auto& [robot_id, _] : m_mapDistanceTraveled) {
        sorted_robot_ids.push_back(robot_id);
    }
    std::sort(sorted_robot_ids.begin(), sorted_robot_ids.end());

    // Write overall statistics
    outFile << "killed " << m_unKilledByEnemy << "\n";
    outFile << "reached_target " << m_unReachedTarget << "\n";
    outFile << "ticks " << CSimulator::GetInstance().GetSpace().GetSimulationClock() << "\n";

    // Write distance traveled
    outFile << "Distance Traveled:\n";
    for (const std::string& robot_id : sorted_robot_ids) {
        outFile << robot_id << ": " << m_mapDistanceTraveled[robot_id] << "\n";
    }

    // // *** Collision Tracking Method (Default) ***
    // // Uncomment this block if you are using collision tracking
    // outFile << "Collision Count:\n";
    // for (const std::string& robot_id : sorted_robot_ids) {
    //     outFile << robot_id << ": " << m_mapCollisionCount[robot_id] << "\n";
    // }
    // outFile << "Total Collisions: " << m_unTotalCollisions << "\n";

    // *** Robot Removal Method ***
    // Uncomment this block if you are using robot removal
    outFile << "Robots removed by collision: " << m_unKilledByEnemy << "\n";
    outFile << "Robots reached the target: " << m_unReachedTarget << "\n";

    outFile.close();
    LOG << "Results saved to results.txt" << std::endl;
}


bool CFlockingLoopFunctions::IsExperimentFinished() {
    // Get all entities of type "e-puck2"
    CSpace::TMapPerType cEpuckBots = GetSpace().GetEntitiesByType("e-puck2");
    int robot_count = 0;

    // Count the number of non-enemy robots still present in the simulation
    for (auto it = cEpuckBots.begin(); it != cEpuckBots.end(); ++it) {
        CEPuck2Entity& cEpuckBot = *any_cast<CEPuck2Entity*>(it->second);
        if (cEpuckBot.GetId().find("enemy_robot") == std::string::npos) {
            ++robot_count;
        }
    }

    // Log the remaining robots for debugging purposes
    LOG << "Number of remaining robots (excluding enemies): " << robot_count << std::endl;

    // Stop the simulation if no robots (excluding enemies) are left
    if (robot_count == 0) {
        LOG << "All non-enemy robots removed; stopping simulation." << std::endl;
        return true;
    }

    return false;
}


void CFlockingLoopFunctions::RemoveRobot(const std::string& strRobotId) {
    try {
        CEntity& cEntity = GetSpace().GetEntity(strRobotId);
        CallEntityOperation<CSpaceOperationRemoveEntity, CSpace, void>(GetSpace(), cEntity);
    } catch (CARGoSException& ex) {
        LOGERR << "Error removing robot '" << strRobotId << "': " << ex.what() << std::endl;
    }
}

REGISTER_LOOP_FUNCTIONS(CFlockingLoopFunctions, "flocking_loop_functions")
