/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_light_default_sensor.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>

#include "epuck2_light_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static CRange<SInt32> UNIT(0, 4095);

   /****************************************/
   /****************************************/

   CEPuck2LightDefaultSensor::CEPuck2LightDefaultSensor() :
      m_pcLightEntity(NULL),
      m_pcControllableEntity(NULL),
      m_bShowRays(false),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()) {}

   /****************************************/
   /****************************************/

   void CEPuck2LightDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcLightEntity = &(c_entity.GetComponent<CLightSensorEquippedEntity>("light_sensors"));
         m_pcLightEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the light default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LightDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_EPuck2LightSensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise level */
         Real fNoiseLevel = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
         if(fNoiseLevel < 0.0f) {
            THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the light sensor");
         }
         else if(fNoiseLevel > 0.0f) {
            m_bAddNoise = true;
            m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         m_tReadings.resize(m_pcLightEntity->GetNumSensors());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default light sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LightDefaultSensor::Update() {
      /* Erase readings */
      for(size_t i = 0; i < m_tReadings.size(); ++i)  m_tReadings[i].Value = 4095;
      /* Ray used for scanning the environment for obstacles */
      CRay3 cScanningRay;
      CVector3 cRayStart;
      CVector3 cSensorToLight;
      /* Buffers to contain data about the intersection */
      SEmbodiedEntityIntersectionItem sIntersection;
      /* Get the map of light entities */
      CSpace::TMapPerTypePerId::iterator itLights = m_cSpace.GetEntityMapPerTypePerId().find("light");
      if (itLights != m_cSpace.GetEntityMapPerTypePerId().end()) {
         CSpace::TMapPerType& mapLights = itLights->second;
         /* Go through the sensors */
         for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
            /* Set ray start */
            cRayStart = m_pcLightEntity->GetSensor(i).Position;
            cRayStart.Rotate(m_pcLightEntity->GetSensor(i).Anchor.Orientation);
            cRayStart += m_pcLightEntity->GetSensor(i).Anchor.Position;
            /* Go through all the light entities */
            Real fReading = 0.0;
            for(CSpace::TMapPerType::iterator it = mapLights.begin();
                it != mapLights.end();
                ++it) {
               /* Get a reference to the light */
               CLightEntity& cLight = *any_cast<CLightEntity*>(it->second);
               /* Consider the light only if it has non zero intensity */
               if(cLight.GetIntensity() > 0.0f) {
                  /* Set ray end to light position */
                  cScanningRay.Set(cRayStart, cLight.GetPosition());
                  if (cScanningRay.GetLength() <=  m_pcLightEntity->GetSensor(i).Direction.Length()) {
                      /* Check occlusions */
                      if(! GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                                    cScanningRay)) {
                         /* No occlusion, the light is visibile */
                         if(m_bShowRays) {
                            m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
                         }
                         /* Calculate reading */
                         cScanningRay.ToVector(cSensorToLight);
                         fReading += CalculateReading(cSensorToLight.Length(),
                                                            cLight.GetIntensity());
                      } else {
                         /* There is an occlusion, the light is not visible */
                         if(m_bShowRays) {
                            m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                                         sIntersection.TOnRay);
                            m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
                         }
                      }
                  }
               }
            }
            /* Apply noise to the sensor */
            if(m_bAddNoise) {
               fReading += m_pcRNG->Uniform(m_cNoiseRange);
            }
            /* Trunc the reading between 0 and 4095 */
            fReading = (1.0 - fReading) * 4095.0;
            m_tReadings[i].Value = Round(fReading);
            UNIT.TruncValue(m_tReadings[i].Value);
         }
      } else {
         /* There are no lights in the environment */
         if(m_bAddNoise) {
            /* Go through the sensors */
            for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
               /* Apply noise to the sensor */
               m_tReadings[i].Value += Round(4095.0 * m_pcRNG->Uniform(m_cNoiseRange));
               /* Trunc the reading between 0 and 1 */
               UNIT.TruncValue(m_tReadings[i].Value);
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LightDefaultSensor::Reset() {
      for(UInt32 i = 0; i < GetReadings().size(); ++i) {
         m_tReadings[i].Value = 4095;
      }
   }

   /****************************************/
   /****************************************/

   Real CEPuck2LightDefaultSensor::CalculateReading(Real f_distance, Real f_intensity) {
      return ((f_intensity / 13.3333333333333) * (f_intensity / 13.3333333333333)) / (f_distance * f_distance);
   }

    /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEPuck2LightDefaultSensor,
                   "epuck2_light", "default",
                   "Daniel H. Stolfi based on Carlo Pinciroli's work",
                   "1.0",
                   "The EPuck 2 generic light sensor.",

                   "This sensor accesses a set of light sensors. The sensors all return a value\n"
                   "between 0 and 4095, where 4095 means nothing within range and 0 means strong light.\n"
                   "Values between 0 and 4095 depend on the distance of the perceived light.\n"
                   "Each reading R is calculated with R=4095*(1-(I/x)^2), where x is the distance between\n"
                   "a sensor and the light, and I is the reference intensity of the\n"
                   "perceived light. The reference intensity corresponds to the minimum distance at\n"
                   "which the light saturates a sensor. The reference intensity depends on the\n"
                   "individual light, and it is set with the \"intensity\" attribute of the light\n"
                   "entity. In case multiple lights are present in the environment, each sensor\n"
                   "reading is calculated as the sum of the individual readings due to each light.\n"
                   "In other words, light wave interference is not taken into account. In\n"
                   "controllers, you must include the ci_epuck2_light_sensor.h header.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <light implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "OPTIONAL XML CONFIGURATION\n\n"

                   "It is possible to draw the rays shot by the light sensor in the OpenGL\n"
                   "visualization. This can be useful for sensor debugging but also to understand\n"
                   "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
                   "they are not obstructed and in purple when they are. In case a ray is\n"
                   "obstructed, a black dot is drawn where the intersection occurred.\n"
                   "To turn this functionality on, add the attribute \"show_rays\" as in this\n"
                   "example:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <light implementation=\"default\"\n"
                   "                   show_rays=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "It is possible to add uniform noise to the sensors, thus matching the\n"
                   "characteristics of a real robot better. This can be done with the attribute\n"
                   "\"noise_level\", whose allowed range is in [-1,1] and is added to the calculated\n"
                   "reading. The final sensor reading is always normalized in the [0-4095] range.\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <light implementation=\"default\"\n"
                   "                   noise_level=\"0.1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "OPTIMIZATION HINTS\n\n"

                   "1. For small swarms, enabling the light sensor (and therefore causing ARGoS to\n"
                   "   update its readings each timestep) unconditionally does not impact performance too\n"
                   "   much. For large swarms, it can impact performance, and selectively\n"
                   "   enabling/disabling the light sensor according to when each individual robot needs it\n"
                   "   (e.g., only when it is returning to the nest from foraging) can increase performance\n"
                   "   by only requiring ARGoS to update the readings for a robot on the timesteps will be\n"
                   "   used.\n",

                   "Usable"
		  );

}
