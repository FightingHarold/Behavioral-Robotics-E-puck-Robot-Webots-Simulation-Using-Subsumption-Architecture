This project implements an autonomous e-puck robot in the Webots simulator using a subsumption architecture inspired by Rodney Brooks’ classical work on layered, parallel behaviors. The robot dynamically prioritizes essential survival tasks like finding food, water, and resting, while avoiding obstacles and interacting with visual landmarks such as beacons and photo frames

What is Webots?

• A professional mobile robot simulation software package.

• offers a rapid prototyping environment that allows the user to create 3D
virtual worlds with physics properties such as mass, joints, and friction
coefficients, etc

• The user can add simple passive objects, or mobile robots, which have different
locomotion schemes (wheeled robots, legged robots, or flying robots) with
several sensor and actuator devices, such as distance sensors, drive
wheels, cameras, motors, touch sensors, emitters, receivers, etc.

#Quick Start:

Webots Installation Website Link: https://cyberbotics.com
File → Open World → behavioral_arena.wbt
1. Right-click empty space → Insert Robot → e-puck
2. OR double-click the existing e-puck robot in the scene tree
Robot (e-puck) → controller → Edit controller

TroubleShooting:
"Robot initialized." → SUCCESS
No print? → Check sensor names match exactly (ps0-ps7, gs0)
Crash? → Missing sensors/motors in world file


# Key Features

- Utilizes a 10-layer subsumption hierarchy to manage diverse behaviors, emphasizing real-time sensor-motor coupling.
- Adapts to environmental conditions, including resource patches (food and water), lighting variations, and both dynamic and static obstacles.
- Employs ground sensors, distance sensors, a camera, and GPS for perception and navigation.
- Prioritizes critical needs (tiredness, hunger, thirst) and environmental hazards via layered behavior arbitration.
- Enables autonomous exploration and survival within a complex and uncertain simulated environment.

<img width="519" height="324" alt="image" src="https://github.com/user-attachments/assets/fe8317b6-a21f-4cff-a958-9febbc74dc1f" />

My Final Webots With E-Puck Robot in the World



<img width="551" height="776" alt="image" src="https://github.com/user-attachments/assets/f498dd8a-ded6-474d-92d3-b61914d62eff" />

Subsumption Architecture



#Results:

Robot autonomously survives 5-10 minutes, completes all 10 behaviors, then gracefully terminates:  
"I had a really good life. I am tired and gonna die. Goodbye."

This modular, biologically-inspired approach demonstrates the effectiveness of subsumption architecture for autonomous survival robotics and opens avenues for further behavioral innovation and complex interaction studies in simulation.

#Key Learnings

Real-time sensor-motor coupling: via layered suppression
Calibration challenges: solved for precise ground sensor thresholds
Smooth behavioral transitions: through internal state + sensor triggers
Instinctive prioritization: mimicking biological responsiveness
