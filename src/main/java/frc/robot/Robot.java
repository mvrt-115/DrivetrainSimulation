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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.VecBuilder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public final static double ROBOT_MASS = 150;
  public final static double MOI = 7.469;
  public final static double GEAR_RATIO = 10;
  public final static double WHEEL_RADIUS = Units.inchesToMeters(3);
  public final static double TRACK_WIDTH = Units.inchesToMeters(20);
  public final static double TICKS_PER_REVOLUTION = 4096;

  private DifferentialDrivetrainSim sim_driveTrain;

  private Encoder lEncoder, rEncoder;
  private EncoderSim lEncoderSim, rEncoderSim;

  private double lMotorOutput, rMotorOutput;
  private double prevTimeStamp;
  private final double p_gain = 0.3, i_gain = 0.00001, d_gain = 0.3;
  private double intergral = 0;
  private double prevPos = 0;
  


  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    sim_driveTrain = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2),       
      GEAR_RATIO,                    
      MOI,                     
      ROBOT_MASS,                    
      WHEEL_RADIUS, 
      TRACK_WIDTH,                  
    
      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    lEncoder = new Encoder(0, 1);
    rEncoder = new Encoder(5, 9);

    lEncoderSim = new EncoderSim(lEncoder);
    rEncoderSim = new EncoderSim(rEncoder);

    lMotorOutput = 0;
    rMotorOutput = 0;

    SmartDashboard.putNumber("P Output", p_gain * 0);
    SmartDashboard.putNumber("I Output", i_gain * intergral);
    SmartDashboard.putNumber("D Output", d_gain * 0);
    SmartDashboard.putNumber("output", 0);
    SmartDashboard.putNumber("pos", lEncoder.getDistance());

    
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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    lEncoder.reset();
    rEncoder.reset();

    prevTimeStamp = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double currPos = lEncoder.getDistance();
    double error = 3 - currPos;
    double deriv = (currPos - prevPos) /  (Timer.getFPGATimestamp() - prevTimeStamp);
    intergral += error;

    double output = (p_gain * error) + (intergral * i_gain) - (deriv * d_gain);

    lMotorOutput = output;
    rMotorOutput = output; 

    prevTimeStamp = Timer.getFPGATimestamp();
    prevPos = currPos;

    SmartDashboard.putNumber("P Output", p_gain * error);
    SmartDashboard.putNumber("I Output", i_gain * intergral);
    SmartDashboard.putNumber("D Output", d_gain * deriv);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("pos", lEncoder.getDistance());


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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

  @Override
  public void simulationPeriodic() {
    sim_driveTrain.setInputs(lMotorOutput * RobotController.getInputVoltage(), 
        rMotorOutput * RobotController.getInputVoltage());

    sim_driveTrain.update(0.02); // update by 20 ms

    lEncoderSim.setDistance(sim_driveTrain.getLeftPositionMeters());
    lEncoderSim.setRate(sim_driveTrain.getLeftVelocityMetersPerSecond());
    rEncoderSim.setDistance(sim_driveTrain.getRightPositionMeters());
    rEncoderSim.setRate(sim_driveTrain.getRightVelocityMetersPerSecond());
  }
}
