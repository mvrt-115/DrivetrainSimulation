/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;



  private Encoder encoder;      
  private double motorOutput;   //motor Output from -1 to 1
  private double target; 


  // Constants 
  private static final double kP = 1;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double robotMass = 68.03;  // 150 pounds to Kg
  private static final double wheelRadius = Units.inchesToMeters(3);
  private static final double trackWidth = Units.inchesToMeters(20);  
  private static final double gearRatio = 10;
  private static final double momentOfInertia = 7.469;
  private static final double ticksPerRevolution = 4096;
 

  //Simulation Tools
  private DCMotor dtMotors;
  private EncoderSim encoderSim;  
  private DifferentialDrivetrainSim drivetrainSim;




  public void robotInit() {

    dtMotors = DCMotor.getFalcon500(2);   // Each side of drivetrain has 2 Falcon500's
    encoder = new Encoder(0, 1);          // Encoder which is on the motor of the Falcon500. Would not use the Encoder object when using code irl
    encoderSim = new EncoderSim(encoder);  // Simulates the encoder readings
    drivetrainSim = new DifferentialDrivetrainSim(dtMotors, gearRatio, momentOfInertia, robotMass, wheelRadius, trackWidth, null);
    encoder.setDistancePerPulse(2 * Math.PI * wheelRadius / ticksPerRevolution  / gearRatio);   // Encoder Reading * Distance Per Pulse gives distance travelled 

    m_robotContainer = new RobotContainer();
    motorOutput = 0;
    target = 3;                 // goal = 3 meters forwards 
  }

 

  public void robotPeriodic() {
    SmartDashboard.putNumber("Motor Output [-1 to 1]", motorOutput);
    SmartDashboard.putNumber("Encoder Reading", encoder.getDistance());
    
  

    CommandScheduler.getInstance().run();
  }

  

  public void disabledInit() {
    motorOutput = 0;
  }

  
  public void disabledPeriodic() {
  }

   public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  double integralLimit = 1; // 1 meter
  double errorSum;
  double lastError = 0;
  double lastTimeStamp = 0; 

  public void autonomousPeriodic() {

    double error = target - encoder.getDistance();



    double dt = Timer.getFPGATimestamp() - lastTimeStamp;   // Should be approximately 0.02s
    double errorRate = (error - lastError)/ dt;

    // This is an Integral Limit, will only add the error if the error is less than 1 meter
    if(error < 1 ){
      errorSum += error * dt;
    }
    
    motorOutput = kP * error + kI * errorSum + kD * errorRate;
    

    // The above is PID and its calculations. This is all explained here: https://youtu.be/jIKBWO7ps0w?t=263
 

    // Makes sure the caluclated percent output falls between [-1, 1]. This is typically done automatically by the Talon, but this code must be written since it is a simulation
    if(motorOutput > 0)
      motorOutput = Math.min(1, motorOutput);
    else if(motorOutput < 1)
      motorOutput = Math.max(-1, motorOutput);
    
    
    lastError = error;
    lastTimeStamp = Timer.getFPGATimestamp();
  }


  public void teleopInit() {
   
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  public void teleopPeriodic() {
    
}

  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public void testPeriodic() {
  }

  public void simulationPeriodic(){

    drivetrainSim.setInputs(motorOutput, motorOutput);    // Gives the drivetrain Sim the calculated motor ouput
    drivetrainSim.update(.02);                            // Simulates 0.02s of time. 0.02s is also how often the roborio/ these methods runs
    
    encoderSim.setDistance(drivetrainSim.getPose().getX());   //Feeds the calculated distance travel into the encoder Sim

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrainSim.getCurrentDrawAmps()));    //Does some funky math to determine the current voltage of the battery based on the current pull of the drivetrain motors


  }
}
