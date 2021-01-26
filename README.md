# DrivetrainSimulation

**Goal: To simulate a drivetrain moving forwards 3 meters in autonomous using PID**

Use the DifferentialDrivetrainSim class as part of the physics simulation in WPILIB
- **documentation:** https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/index.html 
- **example code:** https://github.com/mcm001/allwpilib/tree/state-space-v2/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation 

Write the PID loop from scratch without using the WPI or CTRE libraries for PID
**Resources for PID:** 
1. https://youtu.be/jIKBWO7ps0w
1. https://youtu.be/Z24fSBVJeGs
1. https://blog.wesleyac.com/posts/intro-to-control-part-one-pid


- Put all of your code inside Robot.java
- Use Shuffleboard to plot your motor output and error while tuning PID
- Mess around with the gains for P, I , & D and see how changing each one affects your output

**Additional Information:** 
- Robot Mass: 150 pounds
- Wheel radius: 3 inches
- Track Width: 20 inches
- Gear Ratio: 10:1. (10 rotations of the motor = 1 rotation of the wheels)
- Ticks/Revolution: 4096 (ticks are the unit of the encoder, 4096 encoder ticks = 1 rotation)
- Moment of Inertia: 7.469  
- Drivetrain Motors: 2 Falcon 500's on each side. 

*Make sure to pay attention to units* 

Create a branch of this repository with your name as the branch title. To turn your code in, make a commit titled "DONE" in your branch. 
