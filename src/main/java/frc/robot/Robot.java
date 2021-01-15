// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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
 * 
 * https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/drivesim-tutorial
 */
public class Robot extends TimedRobot {
  private final double GEAR_RATIO = 10;
  private final double MOI = 7.469;
  private final double MASS = 60.0389;
  private final double WHEEL_RADIUS = 3; // 3 inches
  private final double TRACK_WIDTH = 20; // 20 inches
  private final int TICKS = 4096;

  private final double kP = 0.55;
  private final double kI = 0.00001;
  private final double kD = 0.05;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;  

  private DifferentialDrivetrainSim sim;

  private Encoder lEncoder;
  private Encoder rEncoder;
  private EncoderSim lEncoderSim;
  private EncoderSim rEncoderSim;

  private double lMotorOutput;
  private double rMotorOutput;

  private double setPoint;
  private double errorSum;
  private double lastError;
  private double lastTimestamp;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */ 
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    sim = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2),
      GEAR_RATIO, // 10 : 1 gear ratio
      MOI, // MOI
      MASS, // mass of the robot (50lb or ~60kg)
      Units.inchesToMeters(WHEEL_RADIUS), // wheel radius
      Units.inchesToMeters(TRACK_WIDTH), // track width
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) // std deviations
    );

    lEncoder = new Encoder(0, 1);
    rEncoder = new Encoder(2, 3);
    lEncoderSim = new EncoderSim(lEncoder);
    rEncoderSim = new EncoderSim(rEncoder);

    lMotorOutput = 0;
    rMotorOutput = 0;

    setPoint = 10;
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();

    lEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / TICKS);
    rEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / TICKS);
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

    SmartDashboard.putNumber("Encoder Value (left)", lEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Value (right)", rEncoder.getDistance());
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

    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double sensorPosition = lEncoder.getDistance();
    double error = setPoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorRate = (error - lastError) / dt;
    
    errorSum += dt * error;

    double output = kP * error + kI * errorSum + kD * errorRate;

    lMotorOutput = output;
    rMotorOutput = (output); // right motor is inverted

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

    SmartDashboard.putNumber("P Output", kP * error);
    SmartDashboard.putNumber("I Output", kI * errorSum);
    SmartDashboard.putNumber("D Output", kD * errorRate);
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
  public void simulationPeriodic() {
    sim.setInputs(lMotorOutput * RobotController.getInputVoltage(), 
        rMotorOutput * RobotController.getInputVoltage());

    sim.update(0.02); // update by 20 ms

    lEncoderSim.setDistance(sim.getLeftPositionMeters());
    lEncoderSim.setRate(sim.getLeftVelocityMetersPerSecond());
    rEncoderSim.setDistance(sim.getRightPositionMeters());
    rEncoderSim.setRate(sim.getRightVelocityMetersPerSecond());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
