/**
 * @file <flocking_loop_functions.h>
 *
 * @author Eliyahu Houri
 *
 */

#ifndef FLOCKING_LOOP_FUNCTIONS_H_
#define FLOCKING_LOOP_FUNCTIONS_H_

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <string>
#include <unordered_map>
#include <vector>

using namespace argos;

class CFlockingLoopFunctions : public CLoopFunctions {

public:
   CFlockingLoopFunctions();
   virtual ~CFlockingLoopFunctions() {}
   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();
   virtual bool IsExperimentFinished();
   Real target = 7.0;

private:
   // Function to track distance traveled by each robot
   void TrackDistanceTraveled(const std::string& robot_id, const CVector3& current_position);

   // Function to check and count collisions with the enemy
   void CheckCollisionsWithEnemy(const std::string& robot_id, const CVector3& robot_position, const CVector3& enemy_position);

   // Function to check if a robot is close to the target
   bool IsCloseToTarget(CEPuck2Entity& cEpuckBot);

   // Function to remove a robot from the simulation
   void RemoveRobot(const std::string& strRobotId);

   // Data members
   std::vector<CVector2> m_v;                     // Reserved for simulation tracking
   std::unordered_map<std::string, CVector3> m_mapPreviousPositions; // Previous positions of robots
   std::unordered_map<std::string, Real> m_mapDistanceTraveled;      // Distance traveled by each robot
   std::unordered_map<std::string, UInt32> m_mapCollisionCount;      // Collision count per robot
   std::unordered_map<std::string, UInt32> m_mapLastCollisionTick;   // Last collision tick per robot

   unsigned int m_unKilledByEnemy;   // Number of robots killed by the enemy
   unsigned int m_unReachedTarget;   // Number of robots that reached the target
   unsigned int m_unTotalCollisions; // Total number of collisions
   UInt32 m_cooldownTicks;           // Cooldown duration for collision counting
};

#endif /* FLOCKING_LOOP_FUNCTIONS_H_ */
