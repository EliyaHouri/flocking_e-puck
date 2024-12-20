/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_proximity_default_sensor.cpp>
 *
 * @author Daniel H. Stolfi based on the Danesh Tarapore's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

#include "epuck2_proximity_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static CRange<SInt32> UNIT(0, 4095);

   /****************************************/
   /****************************************/

   CEPuck2ProximityDefaultSensor::CEPuck2ProximityDefaultSensor() :
      m_pcEmbodiedEntity(NULL),
      m_pcProximityEntity(NULL),
      m_pcControllableEntity(NULL),
      m_bShowRays(false),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()) {}

   /****************************************/
   /****************************************/

   void CEPuck2ProximityDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcProximityEntity = &(c_entity.GetComponent<CProximitySensorEquippedEntity>("proximity_sensors"));
         m_pcProximityEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the proximity default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2ProximityDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_EPuck2ProximitySensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise level */
         Real fNoiseLevel = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
         if(fNoiseLevel < 0.0f) {
            THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the proximity sensor");
         }
         else if(fNoiseLevel > 0.0f) {
            m_bAddNoise = true;
            m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
            m_pcRNG = CRandom::CreateRNG("argos");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default proximity sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2ProximityDefaultSensor::Update()
   {
      /* Ray used for scanning the environment for obstacles */
      CRay3 cScanningRay;
      CVector3 cRayStart, cRayEnd;
      /* Buffers to contain data about the intersection */
      SEmbodiedEntityIntersectionItem sIntersection;
      /* Go through the sensors */
      for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
         /* Compute ray for sensor i */
         cRayStart = m_pcProximityEntity->GetSensor(i).Offset;
         cRayStart.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
         cRayStart += m_pcProximityEntity->GetSensor(i).Anchor.Position;
         cRayEnd = m_pcProximityEntity->GetSensor(i).Offset;
         cRayEnd += m_pcProximityEntity->GetSensor(i).Direction;
         cRayEnd.Rotate(m_pcProximityEntity->GetSensor(i).Anchor.Orientation);
         cRayEnd += m_pcProximityEntity->GetSensor(i).Anchor.Position;
         cScanningRay.Set(cRayStart,cRayEnd);
         /* Compute reading */
         /* Get the closest intersection */
         Real fReading = 0.0f;
         if(GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                     cScanningRay,
                                                     *m_pcEmbodiedEntity)) {
            /* There is an intersection */
            if(m_bShowRays) {
               m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                            sIntersection.TOnRay);
               m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
            }
            fReading = CalculateReading(cScanningRay.GetDistance(sIntersection.TOnRay));
         } else {
            /* No intersection */
            fReading = 0.0f;

            if(m_bShowRays) {
               m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
            }
         }
         /* Apply noise to the sensor */
         if(m_bAddNoise) {
            fReading += m_pcRNG->Uniform(m_cNoiseRange);
         }
         m_tReadings[i].Value = Round(fReading * 4095);
         /* Trunc the reading between 0 and 4095 */
         UNIT.TruncValue(m_tReadings[i].Value);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2ProximityDefaultSensor::Reset()
   {
      for(UInt32 i = 0; i < GetReadings().size(); ++i)
         m_tReadings[i].Value = 0;
   }

   /****************************************/
   /****************************************/

   Real CEPuck2ProximityDefaultSensor::CalculateReading(Real f_distance)
   {
      Real value = 0.0f;
      if(f_distance <= 0.05) {
          value = 197.8633 * pow(f_distance,2) - 30.47182f * f_distance + 1.011635f;
      }
      CRange<Real>(0.0f,1.0f).TruncValue(value);
      return value;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEPuck2ProximityDefaultSensor,
                   "epuck2_proximity", "default",
                   "Daniel H. Stolfi based on Danesh Tarapore's work",
                   "1.0",
                   "The EPuck2 proximity sensor - where angles of the IR sensors is stored with their usual sensor readings. Useful to quickly compute diffusion vector for e-puck2 robot.",
                   "This sensor accesses a set of proximity sensors. The sensors all return a value\n"
                   "between 0 and 4095, where 0 means nothing within range and 4095 means an external\n"
                   "object is touching the sensor. Values between 0 and 4095 depend on the distance of\n"
                   "the occluding object, and are calculated as value=exp(-distance). In\n"
                   "controllers, you must include the ci_proximity_sensor.h header.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <proximity implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to draw the rays shot by the proximity sensor in the OpenGL\n"
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
                   "        <proximity implementation=\"default\"\n"
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
                   "        <proximity implementation=\"default\"\n"
                   "                   noise_level=\"0.1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",
                   "Usable"
		  );

}
