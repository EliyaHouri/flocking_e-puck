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
#include <string>
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

private:
   std::vector<CVector2> m_v;
   bool IsCloseToTarget(CEPuck2Entity& cEpuckBot);  // Declare IsCloseToTarget function
   void RemoveRobot(const std::string& strRobotId);  // Declare RemoveRobot function
   unsigned int m_unKilledByEnemy;   // Number of robots killed by the enemy
   unsigned int m_unReachedTarget;   // Number of robots that reached the target
};

#endif /* FLOCKING_LOOP_FUNCTIONS_H_ */

