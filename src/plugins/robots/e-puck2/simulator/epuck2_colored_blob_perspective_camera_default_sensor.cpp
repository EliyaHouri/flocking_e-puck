/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_colored_blob_perspective_camera_default_sensor.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */
#include "epuck2_colored_blob_perspective_camera_default_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/media/led_medium.h>
#include "epuck2_camera_equipped_entity.h"


namespace argos {

   /****************************************/
   /****************************************/

   class CEPuck2PerspectiveCameraLEDCheckOperation : public CPositionalIndex<CLEDEntity>::COperation {

   public:

      CEPuck2PerspectiveCameraLEDCheckOperation(
         CCI_ColoredBlobPerspectiveCameraSensor::TBlobList& t_blobs,
         CEPuck2CameraEquippedEntity& c_cam_entity,
         CEmbodiedEntity& c_embodied_entity,
         CControllableEntity& c_controllable_entity,
         bool b_show_rays,
         Real f_noise_std_dev) :
         m_tBlobs(t_blobs),
         m_cCamEntity(c_cam_entity),
         m_cEmbodiedEntity(c_embodied_entity),
         m_cControllableEntity(c_controllable_entity),
         m_bShowRays(b_show_rays),
         m_fNoiseStdDev(f_noise_std_dev),
         m_pcRNG(nullptr) {
            m_pcRootSensingEntity = &m_cEmbodiedEntity.GetRootEntity();
            if(m_fNoiseStdDev > 0.0f) {
               m_pcRNG = CRandom::CreateRNG("argos");
            }
      }

      virtual ~CEPuck2PerspectiveCameraLEDCheckOperation() {
         while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
         }
      }

      virtual bool operator()(CLEDEntity& c_led) {
         /* Process this LED only if it's lit */
         if(c_led.GetColor() != CColor::BLACK) {
            /* Filter out the LEDs belonging to the sensing entity by checking if they share the same parent entity */
            if(m_pcRootSensingEntity == &c_led.GetRootEntity()) return true;
            /* If we are here, it's because the LED must be processed */
            /* Set the end of the ray for occlusion checking */
            m_cOcclusionCheckRay.SetEnd(c_led.GetPosition());
            /* Calculate the vector to LED in the camera-anchor frame of reference */
            m_cLEDRelative = c_led.GetPosition();
            m_cLEDRelative -= m_cCamEntity.GetPosition();
            m_cLEDRelative.Rotate(m_cInvCameraOrient);
            /* Calculate the projection of the LED vector into the camera direction */
            Real fDotProd = m_cLEDRelative.GetX();
            /* The blob is visible if
             * 1. It is within the distance range AND
             * 2. It is within the aperture range AND
             * 3. There are no occlusions
             */
            if(fDotProd < m_cCamEntity.GetRange() &&
               ACos(fDotProd / m_cLEDRelative.Length()) < m_cCamEntity.GetAperture() &&
               !GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem,
                                                         m_cOcclusionCheckRay,
                                                         m_cEmbodiedEntity)) {
               /* The LED is visible */
               /* Calculate the intersection point between the LED ray and the image plane */
               m_cLEDRelative.Normalize();
               m_cLEDRelative *= m_cCamEntity.GetFocalLength() / m_cLEDRelative.GetX();
               /*
                * The image plane is perpendicular to the local X axis
                * Y points to the left, Z up, the origin is in the image center
                * To find the pixel (i,j), we need to flip both Y and Z, and translate the origin
                * So that the origin is up-left, the i axis goes to the right, and the j axis goes down
                */
               SInt32 nI =
                  static_cast<SInt32>(- m_cCamEntity.GetImagePxWidth() /
                  m_cCamEntity.GetImageMtWidth() *
                  (m_cLEDRelative.GetY() -
                   m_cCamEntity.GetImageMtWidth() * 0.5f));
               SInt32 nJ =
                  static_cast<SInt32>(- m_cCamEntity.GetImagePxHeight() /
                  m_cCamEntity.GetImageMtHeight() *
                  (m_cLEDRelative.GetZ() -
                   m_cCamEntity.GetImageMtHeight() * 0.5f));
               /* Make sure (i,j) is within the limits */
               if((nI >= m_cCamEntity.GetImagePxWidth() || nI < 0) ||
                  (nJ >= m_cCamEntity.GetImagePxHeight() || nJ < 0))
                  return true;
               /* Add new blob */
               m_tBlobs.push_back(
                  new CCI_ColoredBlobPerspectiveCameraSensor::SBlob(
                     c_led.GetColor(), nI, nJ));
               /* Draw ray */
               if(m_bShowRays) {
                  m_cControllableEntity.AddCheckedRay(
                     false,
                     CRay3(m_cCamEntity.GetPosition(),
                           c_led.GetPosition()));
               }
            }
         }
         return true;
      }
      
      void Setup() {
         /* Erase blobs */
         while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
         }
         /* Reset ray start */
         m_cOcclusionCheckRay.SetStart(m_cCamEntity.GetPosition());
         /* Calculate inverse of camera orientation */
         m_cInvCameraOrient = m_cCamEntity.GetOrientation().Inverse();
      }
      
   private:
      
      CCI_ColoredBlobPerspectiveCameraSensor::TBlobList& m_tBlobs;
      CEPuck2CameraEquippedEntity& m_cCamEntity;
      CEmbodiedEntity& m_cEmbodiedEntity;
      CControllableEntity& m_cControllableEntity;
      CQuaternion m_cInvCameraOrient;
      bool m_bShowRays;
      CEntity* m_pcRootSensingEntity;
      CRadians m_cTmp1, m_cTmp2;
      CVector3 m_cLEDRelative;
      SEmbodiedEntityIntersectionItem m_sIntersectionItem;
      CRay3 m_cOcclusionCheckRay;
      Real m_fNoiseStdDev;
      CRandom::CRNG* m_pcRNG;
   };

   /****************************************/
   /****************************************/

   CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::CEPuck2ColoredBlobPerspectiveCameraDefaultSensor() :
      m_pcCamEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcLEDIndex(nullptr),
      m_pcEmbodiedIndex(nullptr),
      m_pcOperation(nullptr),
      m_bShowRays(false) {
   }

   /****************************************/
   /****************************************/

   CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::~CEPuck2ColoredBlobPerspectiveCameraDefaultSensor() {
   }

   /****************************************/
   /****************************************/

   void CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      /* Get omndirectional camera equipped entity */
      m_pcCamEntity = &(c_entity.GetComponent<CEPuck2CameraEquippedEntity>("perspective_camera"));
      /* Get controllable entity */
      m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
      /* Get embodied entity */
      m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
   }

   /****************************************/
   /****************************************/

   void CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_ColoredBlobPerspectiveCameraSensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise */
         Real fNoiseStdDev = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_std_dev", fNoiseStdDev, fNoiseStdDev);
         /* Get LED medium from id specified in the XML */
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         m_pcLEDIndex = &(CSimulator::GetInstance().GetMedium<CLEDMedium>(strMedium).GetIndex());
         /* Create check operation */
         m_pcOperation = new CEPuck2PerspectiveCameraLEDCheckOperation(
            m_sReadings.BlobList,
            *m_pcCamEntity,
            *m_pcEmbodiedEntity,
            *m_pcControllableEntity,
            m_bShowRays,
            fNoiseStdDev);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the colored blob perspective camera default sensor", ex);
      }
      /* sensor is disabled by default */
      Disable();
   }

   /****************************************/
   /****************************************/

   void CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::Update() {
      /* sensor is disabled--nothing to do */
      if (IsDisabled()) {
        return;
      }
      /* Increase data counter */
      ++m_sReadings.Counter;
      /* Prepare the operation */
      m_pcOperation->Setup();
      /* Calculate the sensing box */
      Real fHalfRange  = m_pcCamEntity->GetRange() * 0.5f;
      Real fHalfSide   = fHalfRange * Tan(m_pcCamEntity->GetAperture());
      /* Box center */
      CVector3 cCenter(fHalfRange, 0.0f, 0.0f);
      cCenter.Rotate(m_pcCamEntity->GetOrientation());
      cCenter += m_pcCamEntity->GetPosition();
      /* Box half size */
      CVector3 cCorner(fHalfRange, fHalfSide, fHalfSide);
      cCorner.Rotate(m_pcCamEntity->GetOrientation());
      CVector3 cHalfSize(
         Abs(cCorner.GetX()),
         Abs(cCorner.GetY()),
         Abs(cCorner.GetZ()));
      /* Go through LED entities in box range */
      m_pcLEDIndex->ForEntitiesInBoxRange(
         cCenter, cHalfSize, *m_pcOperation);
   }

   /****************************************/
   /****************************************/

   void CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::Reset() {
      m_sReadings.Counter = 0;
      m_sReadings.BlobList.clear();
   }

   /****************************************/
   /****************************************/

   void CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::Destroy() {
      delete m_pcOperation;
   }

   /****************************************/
   /****************************************/

   void CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::Enable() {
      m_pcCamEntity->Enable();
      CCI_Sensor::Enable();

   }

   /****************************************/
   /****************************************/

   void CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::Disable() {
      m_pcCamEntity->Disable();
      CCI_Sensor::Disable();

   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEPuck2ColoredBlobPerspectiveCameraDefaultSensor,
                   "epuck2_colored_blob_perspective_camera", "default",
                   "Daniel H. Stolfi based on Carlo Pinciroli's work",
                   "1.0",

                   "A generic perspective camera sensor to detect colored blobs.",
                   "This sensor accesses an perspective camera that detects colored blobs. The\n"
                   "sensor returns a list of blobs, each defined by a color and a position with\n"
                   "respect to the robot reference point on the ground. In controllers, you must\n"
                   "include the ci_colored_blob_perspective_camera_sensor.h header.\n\n"

                   "This sensor is disabled by default, and must be enabled before it can be\n"
                   "used.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <colored_blob_perspective_camera implementation=\"default\"\n"
                   "                                         medium=\"leds\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "The 'medium' attribute must be set to the id of the leds medium declared in the\n"
                   "<media> section.\n\n"

                   "OPTIONAL XML CONFIGURATION\n\n"

                   "It is possible to draw the rays shot by the camera sensor in the OpenGL\n"
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
                   "        <colored_blob_perspective_camera implementation=\"default\"\n"
                   "                                         medium=\"leds\" />\n"
                   "                                         show_rays=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "It is possible to add uniform noise to the blobs, thus matching the\n"
                   "characteristics of a real robot better. This can be done with the attribute\n"
                   "\"noise_std_dev\".\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <colored_blob_perspective_camera implementation=\"default\"\n"
                   "                                         medium=\"leds\" />\n"
                   "                                         noise_std_dev=\"0.1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n",

                   "Usable"
      );

}
