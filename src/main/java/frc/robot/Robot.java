// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N7;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //2 Falcon 500 motors on each side of the drivetrain.
  private DCMotor motors = DCMotor.getFalcon500(2);

  //Gearing reduction of the robot
  private static final double GEAR_REDUCTION = 10.0D;

  //Moment of Inertia of the robot
  private static final double MOMENT_OF_INERTIA = 7.469D;

  //Mass of the robot in kilograms
  private static final double ROBOT_MASS = 68.0388554D;

  //Radius of the wheels in meters
  private static final double WHEEL_RADIUS = Units.inchesToMeters(3);

  //Track width of the wheels(distance between left and right wheels)
  private static final double TRACK_WIDTH = Units.inchesToMeters(20);

  //The standard deviations for measurement noise: 
  //x and y: 0.001 m
  //heading: 0.001 rad
  //l and r velocity: 0.1 m/s
  //l and r position: 0.005 m
  private static final Vector<N7> STANDARD_DEVIATIONS = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);

  private Encoder leftEncoder = new Encoder(0, 1);
  private Encoder rightEncoder = new Encoder(2, 3);
  
  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

  private DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(motors, GEAR_REDUCTION, MOMENT_OF_INERTIA, ROBOT_MASS, WHEEL_RADIUS, TRACK_WIDTH, STANDARD_DEVIATIONS);

  private double sumErrors, lastError, lastTime;

  private static final double kP = 0.45, kD = 0.14, kI = 0.00001;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / 4096);
    rightEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / 4096);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    driveSim.setInputs(0, 0);
    sumErrors = lastError = lastTime = 0;
    lastTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    double error = 3.0 - driveSim.getLeftPositionMeters();
    double time = Timer.getFPGATimestamp();

    double deriv = (error - lastError)/(time - lastTime);
        
    sumErrors += error;

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("SumErrors", sumErrors);
    SmartDashboard.putNumber("Derivitive", deriv);

    double output = kP * error + kI * sumErrors + kD * deriv; 

    SmartDashboard.putNumber("Output", output);

    driveSim.setInputs(output * RobotController.getInputVoltage(), output * RobotController.getInputVoltage());

    lastError = error;
    lastTime = time;

    // Advance the model by 20 ms.
    driveSim.update(0.02);

    // Update all of our sensors.
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
  }

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
