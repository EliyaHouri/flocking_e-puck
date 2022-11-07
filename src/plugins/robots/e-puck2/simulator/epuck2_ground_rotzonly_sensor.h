/**
 * @file <argos3/plugins/robots/e-puck2/simulator/e-puck2_ground_rotzonly_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_GROUND_ROTZONLY_SENSOR_H
#define EPUCK2_GROUND_ROTZONLY_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CEPuck2GroundRotZOnlySensor;
   class CGroundSensorEquippedEntity;
   class CFloorEntity;
}

#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include "../control_interface/ci_epuck2_ground_sensor.h"

namespace argos {

   class CEPuck2GroundRotZOnlySensor : public CSimulatedSensor,
                                       public CCI_Epuck2GroundSensor {

   public:

      CEPuck2GroundRotZOnlySensor();

      virtual ~CEPuck2GroundRotZOnlySensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to floor entity */
      CFloorEntity* m_pcFloorEntity;

      /** Reference to ground sensor equipped entity associated to this sensor */
      CGroundSensorEquippedEntity* m_pcGroundSensorEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      /** Reference to the space */
      CSpace& m_cSpace;
   };

}

#endif
