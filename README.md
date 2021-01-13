# taltosoid_hardware
Taltos-oid seeks to understand the practicality in augmenting the human body with an additional robotic finger for everyday tasks. Taltos translates to shaman from Hungarian mythology. One potential defining trait of a shaman in the mythos is physical abnormalities such as an additional finger. Ergo Taltos-oid is using robotics to create a shaman through robotics (or just to help stabalize coffee cups).

![Picture shows a gloved hand holding a cup with thumb, index finger, and middle finger. A second cup is being held with the ring finger, pinky, and the supernumerary robotic opposable finger](./documentation/cup-hold.jpg?raw=true)


**DOCUMENTATION IN PROGRESS**


## Motivation
I have been facinated with augmenting the human body with robotic limbs for years now. Nearly as long as I have been introduced to the concepts of transhumanism and post-humanism. There is something captivating about the question "what would be different about me if I were not human". My first attempt at extending human ability is to develop a supernumerary robotic finger that can be used to assist and augment the wearer in grasping tasks during daily living.

There are two schools of thought regarding the "agent-status" of the robotic finger. One view is to treat the robotic finger as a pass-through device extracting signals from EMG or finger tracking without any cognitive processes performed by the finger. This system relies on expertise of the wearer to control the device for the given task. The alternate view is to endow decision making onto the finger, in effect treating it as a traditional robotic agent and thus casting the control problem as human-robot collaboration. I believe the second view is where the most impact with such a device can be found. (This project will explore both since the first view is much easier to get started with).

## Design
I aim to capture the general hardware and software decisions made. Each section will break down the discussion by version (at this time only version one).

### Hardware

#### Version One
The hardware design can be broken into three subsystems:
- Robotic Finger
- Resistive Sense Glove
- Controller Wrist Strap

![Picture shows a gloved hand with the robotic finger attached. Control board (ESP32) is also shown.](./documentation/top-view.jpg?raw=true "Version one hardware")

**Robotic Finger**:


**Resistive Sense Glove**:


**Controller Wrist Strap**:
I build the controller wrist strap to physically hold the glove and finger aligned. Specifically, the wrist strap limits the glove from shifting around whent the user is moving their hand. Unfortuntely this makes it a bit uncomfortable to wear as it can get rather tight.

Electronically, the finger servos connect into a servo control board and the resistive sensors connect into an analog mux. The brains of the system is an ESP32 with arduino firmware.


Below are some of the key improvements needed for version two:

- Improved power system
- Custom, compact controller PCB
- Higher torque, more compact finger design

## Experiments
My goal once I develop version two is to perform a day-long autobiographical study wearing the finger.

### Software

#### Version One

Below are some of the key improvements needed for version two:

- Explore alternative methods for user control (EMG, nIRS, Vision)
- Communication from finger back to wearer
- Environment sensing (3D grasp targeting)
- Collision avoidance, context aware pose
- Augmentaton control strategies
