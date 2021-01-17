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
import edu.wpi.first.wpilibj2.command.Command;
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
  public static final double MASS = 150;
  public static final double WHEEL_RADIUS = 3 * 0.0254;
  public static final double TRACK_WIDTH = 20 * 0.0254;
  public static final double GEAR_RATIO = 10;
  public static final double INERTIA = 7.469;
  public static final double TICKS = 4096;
  public static final double P = 0.2;
  public static final double I = 0.00001;
  public static final double D = 0.05;
  public static final double TRAVEL_DISTANCE = 3.0;

  public static final Vector<N7> STD_DEVS = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
  
  private DifferentialDrivetrainSim sim;

  private Encoder encoderL;
  private Encoder encoderR;

  private EncoderSim encoderLSim;
  private EncoderSim encoderRSim;

  private double outputL;
  private double outputR;
  private double prevTime;
  private double prevPos;
  private double intergral;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    sim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), GEAR_RATIO, INERTIA, MASS, WHEEL_RADIUS, TRACK_WIDTH, STD_DEVS);

    encoderL = new Encoder(0, 1);
    encoderR = new Encoder(2, 3);

    encoderL.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / TICKS);
    encoderR.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / TICKS);

    encoderLSim = new EncoderSim(encoderL);
    encoderRSim = new EncoderSim(encoderR);

    outputL = 0;
    outputR = 0;
    prevPos = 0;

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
    SmartDashboard.putNumber("P Output", 0);
    SmartDashboard.putNumber("I Output", 0);
    SmartDashboard.putNumber("D Output", 0);
    SmartDashboard.putNumber("Output", 0);
    SmartDashboard.putNumber("Position", 0);
  }

  @Override
  public void disabledPeriodic() {
    encoderL.reset();
    encoderR.reset();

    prevTime = Timer.getFPGATimestamp();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    encoderL.reset();
    encoderR.reset();

    prevTime = Timer.getFPGATimestamp();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double pos = encoderL.getDistance();
    double time = Timer.getFPGATimestamp();
    double error = TRAVEL_DISTANCE - pos;
    double der = (pos - prevPos) / (time - prevTime);
    intergral += error;

    outputL = outputR = P * error + I * intergral - D * der;

    prevTime = time;
    prevPos = pos;

    SmartDashboard.putNumber("P Output", P * error);
    SmartDashboard.putNumber("I Output", I * intergral);
    SmartDashboard.putNumber("D Output", D * der);
    SmartDashboard.putNumber("Output", outputL);
    SmartDashboard.putNumber("Position", encoderL.getDistance());
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
    super.simulationPeriodic();

    sim.setInputs(outputL * RobotController.getInputVoltage(), outputR * RobotController.getInputVoltage());

    sim.update(0.02);

    encoderLSim.setDistance(sim.getLeftPositionMeters());
    encoderLSim.setRate(sim.getLeftVelocityMetersPerSecond());
    encoderRSim.setDistance(sim.getRightPositionMeters());
    encoderRSim.setRate(sim.getRightVelocityMetersPerSecond());
  }
}
