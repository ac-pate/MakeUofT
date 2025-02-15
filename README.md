# MakeUofT


Overview:

We are building a holistic aid system for the visually impaired. Features include:

- A multi-functional object detection system using computer vision and a depth camera. By default this is designed to spot obstacles in a user's path, but it can be prompted to recognize any objects that the COCO dataset can handle by voice activation. When the obstacle is spotted, information is relayed back to the user via our haptic feedback system.
- A robust alert bracelet for alerts in case of emergency. When our IMU detects rapid deceleration, it will ask the user using text-to-speech whether to notify the user's emergency contact. In the event of no response it will notify the contact. Also, this emergency contact can be notified with a voice command at any time.

What are we using?
- Intel RealSense D435 depth camera for object detection mounted on a baseball cap
- OpenCV and COCO dataset for computer vision object recognition
- Arduino Nano 33 DLE Rev2 for speech-to-text recognition and IMU controller
