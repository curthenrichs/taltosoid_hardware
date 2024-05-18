# taltosoid_srf
Taltos-oid seeks to understand the practicality in augmenting the human body with an additional robotic finger for everyday tasks. Taltos translates to shaman from Hungarian mythology. One potential defining trait of a shaman in the mythos is physical abnormalities such as an additional finger. Ergo Taltos-oid is using robotics to create a shaman through robotics (or just to help stabilize coffee cups).

![Picture shows a gloved hand holding a cup with thumb, index finger, and middle finger. A second cup is being held with the ring finger, pinky, and the supernumerary robotic opposable finger](./documentation/cup-hold.jpg?raw=true)

## Motivation
I have been fascinated with augmenting the human body with robotic limbs for years now. Nearly as long as I have been introduced to the concepts of transhumanism and post-humanism. There is something captivating about the question "what would be different about me if I were not human". My first attempt at extending human ability is to develop a supernumerary robotic finger that can be used to assist and augment the wearer in grasping tasks during daily living.

There are two schools of thought regarding the "agent-status" of the robotic finger. One view is to treat the robotic finger as a pass-through device extracting signals from EMG or finger tracking without any cognitive processes performed by the finger. This system relies on expertise of the wearer to control the device for the given task. The alternate view is to endow decision making onto the finger, in effect treating it as a traditional robotic agent and thus casting the control problem as social human-robot collaboration. I believe the second view is where the most impact with such a device can be found. (This project will explore both since the first view is much easier to get started with).

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
I designed a 3D printed robotic finger that uses high-torque micro servos to direct drive the joints. This design was primarily done for proof of concept. I suggest that future versions use some kind of linkage in order to keep the weight of the finger at the base. As it currently is, the servos don't have enough torque to hold even medium weighted objects. 

**Resistive Sense Glove**:
For version one I went with a simple-direct control strategy. In order to accomplish this I developed home-made flex sensors and sewed them onto a glove-liner. The sensors were made from two thick conductive threads with flexible resisitive material in between. I then used conductive copper fabric on each end as connective tabs. I covered the copper with conductive paint to adhere the conductive thread with the copper. Lastly, I sewwed thinner conductive thread down the glove to headerpins, again adhering with conductive paint and electrical tape. 

I later discovered while using the glove that sweat breaks down the conductive paint causing intermittent connections. I suggest future designs not use the approach described above. Instead it will be much more reliable to use off-the-shelf flex sensors even if they are less comfortable to wear. 

**Controller Wrist Strap**:
I build the controller wrist strap to physically keep the glove and finger aligned. Specifically, the wrist strap limits the glove from shifting around whent the user is moving their hand. Unfortuntely this makes it a bit uncomfortable to wear as it can get rather tight.

Electronically, the finger servos connect into a servo control board and the resistive sensors connect into an analog mux. The brains of the system is an ESP32 with arduino firmware. I also mounted an IMU at one point though later removed it for a different project.

**Improvement**:
Below are some of the key improvements needed for version two:

- Improved power system
- Custom, compact controller PCB
- Higher torque, more compact finger design

### Software

#### Version One
Version one software has a PC component and the low-level firmware. The PC was responsible for training and executing an ML model that mapped finger flex sensor input into appropriate joint control. The firmware acted as a relay. It captured low-level ADC voltage readings from the flex sensors. It provided direct joint-control to each joint. And it provided IMU readings. Communication was done over WiFi (thanks to the ESP32's capabilities).

I also started developing a ROS layer for this device, partially as a learning exercise and equally as the next step of building verison two. My goal is to build out several control strategies in simulation while working on a good second finger design.

Below are some of the key improvements needed for version two:

- Explore alternative methods for user control (EMG, nIRS, Vision)
- Communication from finger back to wearer (haptic?, animation?)
- Environment sensing (3D grasp targeting)
- Collision avoidance, context aware pose
- Augmentaton control strategies
- (Recently added) LLM control
- Tamagotchi mode

## Experiments
My goal once I develop version two is to perform a day-long autobiographical study wearing the finger. Work can also be done under more strict laboratory experiments investigating grasping tasks.

