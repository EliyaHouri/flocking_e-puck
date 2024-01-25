var searchData=
[
  ['register_5factuator_410',['REGISTER_ACTUATOR',['../epuck2__led__default__actuator_8cpp.html#adc2bb0285a8aa34ce17e7e6d3b0c1771',1,'epuck2_led_default_actuator.cpp']]],
  ['register_5fbattery_5fdischarge_5fmodel_411',['REGISTER_BATTERY_DISCHARGE_MODEL',['../namespaceargos.html#af69ccc3b5511fbec91402634ca0ba4d4',1,'argos::REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelLinear, &quot;linear&quot;)'],['../namespaceargos.html#afb3053127b753cec38559b0364a591d5',1,'argos::REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelApprox, &quot;approx&quot;)'],['../namespaceargos.html#adeb1873daf3b57467a3b278b9ac8f2cf',1,'argos::REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelCubic, &quot;cubic&quot;)'],['../namespaceargos.html#a2f4eb36828536944c2432ad7cb5465f4',1,'argos::REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelSimple, &quot;simple&quot;)']]],
  ['register_5fentity_412',['REGISTER_ENTITY',['../namespaceargos.html#a0302543f8035fcce7db0f159422fbd2e',1,'argos']]],
  ['register_5fqtopengl_5fentity_5foperation_413',['REGISTER_QTOPENGL_ENTITY_OPERATION',['../namespaceargos.html#a79507d35839934dfcf52ddc11ea52ab8',1,'argos::REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawEPuck2Normal, CEPuck2Entity)'],['../namespaceargos.html#a4bf3cf6a5c8b9c92e9abefc721bd1968',1,'argos::REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawEPuck2Selected, CEPuck2Entity)']]],
  ['register_5fsensor_414',['REGISTER_SENSOR',['../namespaceargos.html#ad97f91055fdde814018d716b283aa0a3',1,'argos::REGISTER_SENSOR(CEPuck2EncoderDefaultSensor, &quot;epuck2_encoder&quot;, &quot;default&quot;, &quot;Daniel H. Stolfi based on Carlo Pinciroli&apos;s work&quot;, &quot;1.0&quot;, &quot;The EPuck2 wheel encoder sensor.&quot;, &quot;This sensor provides the value of the wheels encoders\n&quot; &quot;between -32768 and 32768, where 1000 steps represent a complete wheel revolution.\n&quot; &quot;In controllers, you must include the ci_epuck2_encoder_sensor.h header.\n\n&quot; &quot;REQUIRED XML CONFIGURATION\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_encoder implementation=\&quot;default\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;OPTIONAL XML CONFIGURATION\n\n&quot; &quot;It is possible to add uniform noise to the sensors, thus matching the\n&quot; &quot;characteristics of a real robot better. This can be done with the attribute\n&quot; &quot;\&quot;noise_level\&quot;, whose allowed range is in [-1,1] and is added to the calculated\n&quot; &quot;reading. The final sensor reading is always normalised in the [-32768-32768] range.\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_encoder implementation=\&quot;rot_z_only\&quot;\n&quot; &quot;                        noise_level=\&quot;1\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot;, &quot;Usable&quot;)'],['../namespaceargos.html#ac3874865dbbc30ffa7355aebf3f2de00',1,'argos::REGISTER_SENSOR(CEPuck2BatteryDefaultSensor, &quot;epuck2_battery&quot;, &quot;default&quot;, &quot;Daniel H. Stolfi based on the Adhavan Jayabalan&apos;s work&quot;, &quot;1.0&quot;, &quot;A battery level sensor for E-Pucks2 robots.&quot;, &quot;This sensor returns the current battery level of an e-puck2 robot. In\n&quot; &quot;controllers, you must include the ci_battery_sensor.h header.\n\n&quot; &quot;This sensor is enabled by default.\n\n&quot; &quot;REQUIRED XML CONFIGURATION\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_battery implementation=\&quot;default\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;OPTIONAL XML CONFIGURATION\n\n&quot; &quot;It is possible to add uniform noise to the sensor, thus matching the\n&quot; &quot;characteristics of a real robot better. You can add noise through the\n&quot; &quot;attribute &apos;noise_range&apos; as follows:\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_battery implementation=\&quot;default\&quot;\n&quot; &quot;                 noise_range=\&quot;-0.3:0.4\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot;, &quot;Usable&quot;)'],['../namespaceargos.html#aa90c6be3afc5fe09b603bc4527e298c9',1,'argos::REGISTER_SENSOR(CEPuck2ColoredBlobPerspectiveCameraDefaultSensor, &quot;epuck2_colored_blob_perspective_camera&quot;, &quot;default&quot;, &quot;Daniel H. Stolfi based on Carlo Pinciroli&apos;s work&quot;, &quot;1.0&quot;, &quot;A generic perspective camera sensor to detect colored blobs.&quot;, &quot;This sensor accesses an perspective camera that detects colored blobs. The\n&quot; &quot;sensor returns a list of blobs, each defined by a color and a position with\n&quot; &quot;respect to the robot reference point on the ground. In controllers, you must\n&quot; &quot;include the ci_colored_blob_perspective_camera_sensor.h header.\n\n&quot; &quot;This sensor is disabled by default, and must be enabled before it can be\n&quot; &quot;used.\n\n&quot; &quot;REQUIRED XML CONFIGURATION\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;colored_blob_perspective_camera implementation=\&quot;default\&quot;\n&quot; &quot;                                         medium=\&quot;leds\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;The &apos;medium&apos; attribute must be set to the id of the leds medium declared in the\n&quot; &quot;&lt;media&gt; section.\n\n&quot; &quot;OPTIONAL XML CONFIGURATION\n\n&quot; &quot;It is possible to draw the rays shot by the camera sensor in the OpenGL\n&quot; &quot;visualization. This can be useful for sensor debugging but also to understand\n&quot; &quot;what&apos;s wrong in your controller. In OpenGL, the rays are drawn in cyan when\n&quot; &quot;they are not obstructed and in purple when they are. In case a ray is\n&quot; &quot;obstructed, a black dot is drawn where the intersection occurred.\n&quot; &quot;To turn this functionality on, add the attribute \&quot;show_rays\&quot; as in this\n&quot; &quot;example:\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;colored_blob_perspective_camera implementation=\&quot;default\&quot;\n&quot; &quot;                                         medium=\&quot;leds\&quot; /&gt;\n&quot; &quot;                                         show_rays=\&quot;true\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;It is possible to add uniform noise to the blobs, thus matching the\n&quot; &quot;characteristics of a real robot better. This can be done with the attribute\n&quot; &quot;\&quot;noise_std_dev\&quot;.\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;colored_blob_perspective_camera implementation=\&quot;default\&quot;\n&quot; &quot;                                         medium=\&quot;leds\&quot; /&gt;\n&quot; &quot;                                         noise_std_dev=\&quot;0.1\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n&quot;, &quot;Usable&quot;)'],['../namespaceargos.html#afe7d922e4f65b223fd2d156901269524',1,'argos::REGISTER_SENSOR(CEPuck2GroundRotZOnlySensor, &quot;epuck2_ground&quot;, &quot;rot_z_only&quot;, &quot;Daniel H. Stolfi based on Carlo Pinciroli&apos;s work&quot;, &quot;1.0&quot;, &quot;The EPuck2 ground sensor (optimised for 2D).&quot;, &quot;This sensor accesses a set of ground sensors. The sensors all return a value\n&quot; &quot;between 0 and 1023, where 0 means black and 1023 means white. Depending on the type\n&quot; &quot;of ground sensor, readings can be a value in between (grayscale sensors). \n&quot; &quot;In controllers, you must include the ci_epuck2_ground_sensor.h header.\n\n&quot; &quot;REQUIRED XML CONFIGURATION\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_ground implementation=\&quot;rot_z_only\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;OPTIONAL XML CONFIGURATION\n\n&quot; &quot;It is possible to add uniform noise to the sensors, thus matching the\n&quot; &quot;characteristics of a real robot better. This can be done with the attribute\n&quot; &quot;\&quot;noise_level\&quot;, whose allowed range is in [-1,1] and is added to the calculated\n&quot; &quot;reading. The final sensor reading is always normalised in the [0-1] range.\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_ground implementation=\&quot;rot_z_only\&quot;\n&quot; &quot;                       noise_level=\&quot;0.1\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot;, &quot;Usable&quot;)'],['../namespaceargos.html#a2cd4735317cca9d56bbb1921994f9dfe',1,'argos::REGISTER_SENSOR(CEPuck2LightDefaultSensor, &quot;epuck2_light&quot;, &quot;default&quot;, &quot;Daniel H. Stolfi based on Carlo Pinciroli&apos;s work&quot;, &quot;1.0&quot;, &quot;The EPuck 2 generic light sensor.&quot;, &quot;This sensor accesses a set of light sensors. The sensors all return a value\n&quot; &quot;between 0 and 4095, where 4095 means nothing within range and 0 means strong light.\n&quot; &quot;Values between 0 and 4095 depend on the distance of the perceived light.\n&quot; &quot;Each reading R is calculated with R=4095*(1-(I/x)^2), where x is the distance between\n&quot; &quot;a sensor and the light, and I is the reference intensity of the\n&quot; &quot;perceived light. The reference intensity corresponds to the minimum distance at\n&quot; &quot;which the light saturates a sensor. The reference intensity depends on the\n&quot; &quot;individual light, and it is set with the \&quot;intensity\&quot; attribute of the light\n&quot; &quot;entity. In case multiple lights are present in the environment, each sensor\n&quot; &quot;reading is calculated as the sum of the individual readings due to each light.\n&quot; &quot;In other words, light wave interference is not taken into account. In\n&quot; &quot;controllers, you must include the ci_epuck2_light_sensor.h header.\n\n&quot; &quot;REQUIRED XML CONFIGURATION\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;light implementation=\&quot;default\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;OPTIONAL XML CONFIGURATION\n\n&quot; &quot;It is possible to draw the rays shot by the light sensor in the OpenGL\n&quot; &quot;visualization. This can be useful for sensor debugging but also to understand\n&quot; &quot;what&apos;s wrong in your controller. In OpenGL, the rays are drawn in cyan when\n&quot; &quot;they are not obstructed and in purple when they are. In case a ray is\n&quot; &quot;obstructed, a black dot is drawn where the intersection occurred.\n&quot; &quot;To turn this functionality on, add the attribute \&quot;show_rays\&quot; as in this\n&quot; &quot;example:\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;light implementation=\&quot;default\&quot;\n&quot; &quot;                   show_rays=\&quot;true\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;It is possible to add uniform noise to the sensors, thus matching the\n&quot; &quot;characteristics of a real robot better. This can be done with the attribute\n&quot; &quot;\&quot;noise_level\&quot;, whose allowed range is in [-1,1] and is added to the calculated\n&quot; &quot;reading. The final sensor reading is always normalized in the [0-4095] range.\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;light implementation=\&quot;default\&quot;\n&quot; &quot;                   noise_level=\&quot;0.1\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;OPTIMIZATION HINTS\n\n&quot; &quot;1. For small swarms, enabling the light sensor (and therefore causing ARGoS to\n&quot; &quot;   update its readings each timestep) unconditionally does not impact performance too\n&quot; &quot;   much. For large swarms, it can impact performance, and selectively\n&quot; &quot;   enabling/disabling the light sensor according to when each individual robot needs it\n&quot; &quot;   (e.g., only when it is returning to the nest from foraging) can increase performance\n&quot; &quot;   by only requiring ARGoS to update the readings for a robot on the timesteps will be\n&quot; &quot;   used.\n&quot;, &quot;Usable&quot;)'],['../namespaceargos.html#a7920ee0826c7593bcb169217f6984ea8',1,'argos::REGISTER_SENSOR(CEPuck2ProximityDefaultSensor, &quot;epuck2_proximity&quot;, &quot;default&quot;, &quot;Daniel H. Stolfi based on Danesh Tarapore&apos;s work&quot;, &quot;1.0&quot;, &quot;The EPuck2 proximity sensor - where angles of the IR sensors is stored with their usual sensor readings. Useful to quickly compute diffusion vector for e-puck2 robot.&quot;, &quot;This sensor accesses a set of proximity sensors. The sensors all return a value\n&quot; &quot;between 0 and 4095, where 0 means nothing within range and 4095 means an external\n&quot; &quot;object is touching the sensor. Values between 0 and 4095 depend on the distance of\n&quot; &quot;the occluding object, and are calculated as value=exp(-distance). In\n&quot; &quot;controllers, you must include the ci_proximity_sensor.h header.\n\n&quot; &quot;REQUIRED XML CONFIGURATION\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;proximity implementation=\&quot;default\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;OPTIONAL XML CONFIGURATION\n\n&quot; &quot;It is possible to draw the rays shot by the proximity sensor in the OpenGL\n&quot; &quot;visualization. This can be useful for sensor debugging but also to understand\n&quot; &quot;what&apos;s wrong in your controller. In OpenGL, the rays are drawn in cyan when\n&quot; &quot;they are not obstructed and in purple when they are. In case a ray is\n&quot; &quot;obstructed, a black dot is drawn where the intersection occurred.\n&quot; &quot;To turn this functionality on, add the attribute \&quot;show_rays\&quot; as in this\n&quot; &quot;example:\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;proximity implementation=\&quot;default\&quot;\n&quot; &quot;                   show_rays=\&quot;true\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;It is possible to add uniform noise to the sensors, thus matching the\n&quot; &quot;characteristics of a real robot better. This can be done with the attribute\n&quot; &quot;\&quot;noise_level\&quot;, whose allowed range is in [-1,1] and is added to the calculated\n&quot; &quot;reading. The final sensor reading is always normalized in the [0-4095] range.\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;proximity implementation=\&quot;default\&quot;\n&quot; &quot;                   noise_level=\&quot;0.1\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot;, &quot;Usable&quot;)'],['../namespaceargos.html#a2a03b1930ec5da70e14e8c01008b48c0',1,'argos::REGISTER_SENSOR(CEPuck2TOFDefaultSensor, &quot;epuck2_tof&quot;, &quot;default&quot;, &quot;Daniel H. Stolfi based on Danesh Tarapore&apos;s work&quot;, &quot;1.0&quot;, &quot;The EPuck2 ToF (Time of Fight) sensor.&quot;, &quot;This sensor gets the distance to a possible obstacle in front of the robot. The return value\n&quot; &quot;is between 20 and 2000, where 20 is the minimum measured distance and 2000 means that no object\n&quot; &quot;is detected in 2 metres range.\n&quot; &quot;Values between 10 and 2000 are the distance in millimetres.\n\n&quot; &quot;REQUIRED XML CONFIGURATION\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_tof implementation=\&quot;default\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;OPTIONAL XML CONFIGURATION\n\n&quot; &quot;It is possible to draw the ray shot by the proximity sensor in the OpenGL\n&quot; &quot;visualisation. This can be useful for sensor debugging but also to understand\n&quot; &quot;what&apos;s wrong in your controller. In OpenGL, the ray is drawn in cyan when\n&quot; &quot;it is not obstructed and in purple when it is. In case the ray is\n&quot; &quot;obstructed, a black dot is drawn where the intersection occurred.\n&quot; &quot;To turn this functionality on, add the attribute \&quot;show_rays\&quot; as in this\n&quot; &quot;example:\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_tof implementation=\&quot;default\&quot;\n&quot; &quot;                    show_rays=\&quot;true\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot; &quot;It is possible to add uniform noise to the sensor, thus matching the\n&quot; &quot;characteristics of a real robot better. This can be done with the attribute\n&quot; &quot;\&quot;noise_level\&quot;, whose allowed range is in [-1,1] and is added to the calculated\n&quot; &quot;reading. The final sensor reading is always normalised in the [10-2000] range.\n\n&quot; &quot;  &lt;controllers&gt;\n&quot; &quot;    ...\n&quot; &quot;    &lt;my_controller ...&gt;\n&quot; &quot;      ...\n&quot; &quot;      &lt;sensors&gt;\n&quot; &quot;        ...\n&quot; &quot;        &lt;epuck2_tof implementation=\&quot;default\&quot;\n&quot; &quot;                    noise_level=\&quot;0.1\&quot; /&gt;\n&quot; &quot;        ...\n&quot; &quot;      &lt;/sensors&gt;\n&quot; &quot;      ...\n&quot; &quot;    &lt;/my_controller&gt;\n&quot; &quot;    ...\n&quot; &quot;  &lt;/controllers&gt;\n\n&quot;, &quot;Usable&quot;)']]],
  ['register_5fstandard_5fdynamics2d_5foperations_5fon_5fentity_415',['REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY',['../namespaceargos.html#a3b4176989e15ba988f184af72b659ea8',1,'argos']]],
  ['register_5fstandard_5fspace_5foperations_5fon_5fcomposable_416',['REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE',['../namespaceargos.html#a738588da27a7671d7fec810d0b07a729',1,'argos::REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CEPuck2Entity)'],['../namespaceargos.html#a2d9c6069b84f65cf3541912bd5119e00',1,'argos::REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CEPuck2LEDEquippedEntity)']]],
  ['register_5fstandard_5fspace_5foperations_5fon_5fentity_417',['REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY',['../namespaceargos.html#a40a894aa231e4f721347467e52b30808',1,'argos::REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2TOFEquippedEntity)'],['../namespaceargos.html#ab29c8030b7b98bf1ebf3a8e1ab34c853',1,'argos::REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2BatteryEquippedEntity)'],['../namespaceargos.html#aee742c9fc128dc32fde9e8720bec465e',1,'argos::REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2CameraEquippedEntity)'],['../namespaceargos.html#a2ea434447dbe6f9070a0af48e54506f4',1,'argos::REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2EncoderEquippedEntity)']]],
  ['renderbody_418',['RenderBody',['../classargos_1_1CQTOpenGLEPuck2.html#a3f430684dec18ef2cf41e553951d9ad8',1,'argos::CQTOpenGLEPuck2']]],
  ['renderchassis_419',['RenderChassis',['../classargos_1_1CQTOpenGLEPuck2.html#a283db35063c8ccf8e056115965af8334',1,'argos::CQTOpenGLEPuck2']]],
  ['renderfrontled_420',['RenderFrontLED',['../classargos_1_1CQTOpenGLEPuck2.html#a07a4987838f9d9f51d8afcd82e3c68a3',1,'argos::CQTOpenGLEPuck2']]],
  ['rendergap_421',['RenderGap',['../classargos_1_1CQTOpenGLEPuck2.html#a879b650879b1c4b5e04788291af65ff4',1,'argos::CQTOpenGLEPuck2']]],
  ['renderled_422',['RenderLED',['../classargos_1_1CQTOpenGLEPuck2.html#ac8b62fc6fe71d82d3cc7e0204e25b947',1,'argos::CQTOpenGLEPuck2']]],
  ['renderwheel_423',['RenderWheel',['../classargos_1_1CQTOpenGLEPuck2.html#a1e2165898ea36f1d061a72c99489f4fa',1,'argos::CQTOpenGLEPuck2']]],
  ['reset_424',['Reset',['../classargos_1_1CEPuck2BatteryDischargeModel.html#abd85fd8d831e2c08310c091abb29c8f3',1,'argos::CEPuck2BatteryDischargeModel::Reset()'],['../classargos_1_1CDynamics2DEPuck2Model.html#a99ff9e46eaadc8bef5320afe2c9ef294',1,'argos::CDynamics2DEPuck2Model::Reset()'],['../classargos_1_1CEPuck2BatteryDefaultSensor.html#afbb84e5cd58d1ca717695a3bdd9b7cdc',1,'argos::CEPuck2BatteryDefaultSensor::Reset()'],['../classargos_1_1CEPuck2TOFDefaultSensor.html#a2dfae7c747a7b9456afdf095e7caffa9',1,'argos::CEPuck2TOFDefaultSensor::Reset()'],['../classargos_1_1CEPuck2ColoredBlobPerspectiveCameraDefaultSensor.html#a10fb25f22e39d285670a0c5ab57828b5',1,'argos::CEPuck2ColoredBlobPerspectiveCameraDefaultSensor::Reset()'],['../classargos_1_1CEPuck2EncoderDefaultSensor.html#a6425b089221740b545686e90378f7bc6',1,'argos::CEPuck2EncoderDefaultSensor::Reset()'],['../classargos_1_1CEPuck2Entity.html#ad6ba92ee0e9404f27b9eaa34b81c271b',1,'argos::CEPuck2Entity::Reset()'],['../classargos_1_1CEPuck2GroundRotZOnlySensor.html#a1bab6edc38c56abf99873147d83c572e',1,'argos::CEPuck2GroundRotZOnlySensor::Reset()'],['../classargos_1_1CEPuck2LEDsDefaultActuator.html#aca397549964a04632243a6a665652504',1,'argos::CEPuck2LEDsDefaultActuator::Reset()'],['../classargos_1_1CEPuck2LEDEquippedEntity.html#a78e3f634b5666f78da24d9c66014049f',1,'argos::CEPuck2LEDEquippedEntity::Reset()'],['../classargos_1_1CEPuck2LightDefaultSensor.html#aa8b07b7db0a6216fd4aba428068aef3e',1,'argos::CEPuck2LightDefaultSensor::Reset()'],['../classargos_1_1CEPuck2ProximityDefaultSensor.html#ad30b756a076e3f72b8b1ceacf2581086',1,'argos::CEPuck2ProximityDefaultSensor::Reset()']]]
];
