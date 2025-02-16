# SightSync


Overview:

We are building a holistic aid system for visually impaired people. It consists of a multi-functional object detection system using computer vision and a depth camera. By default this is designed to spot obstacles in a user's path, but it can be prompted to recognize any objects that the COCO dataset can handle via voice activated prompts. When a nearby obstacle or desired object is spotted, information is relayed back to the user via our robust haptic feedback system. Peripherals include a hip-mounted alert pack for in case of emergency. When our IMU detects rapid deceleration, our microcontroller becomes a BLE beacon that can connect to any phone in roughly a 30-50 meter radius. In the event of no response it will notify the contact.

What are we using?
- Intel RealSense D435 depth camera for object detection mounted on a baseball cap
- OpenCV and COCO dataset for computer vision object recognition
- NVIDIA Jetson Nano microcontroller
- Arduino Nano 33 DLE Rev2 for speech-to-text recognition, IMU controls, and BLE emergency signal
