// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String defaultAuto = "Default";
  private static final String customAuto = "My Auto";
  private String autoSelected;
  private final LoggedDashboardChooser<String> chooser =
      new LoggedDashboardChooser<>("Auto Choices");
  // Assuming LoggedDashboardNumber is a class and motorCount is the number of motors
  LoggedDashboardNumber intake = new LoggedDashboardNumber("Intake", 0.0);
  LoggedDashboardNumber shooterup = new LoggedDashboardNumber("shooterUp", 0.0);
  LoggedDashboardNumber shooterdown = new LoggedDashboardNumber("shooterDown", 0.0);
  LoggedDashboardBoolean linkedShooter = new LoggedDashboardBoolean("link", false);
  LoggedDashboardBoolean zero = new LoggedDashboardBoolean("zero", false);
  SparkPIDController[] pidControllers;
  CANSparkMax[] motors = {
    new CANSparkMax(40, MotorType.kBrushless),
    new CANSparkMax(41, MotorType.kBrushless),
    new CANSparkMax(42, MotorType.kBrushless),
  };
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        for (int i = 0; i < motors.length; i++) {
          // motors[i].restoreFactoryDefaults();
          // motors[i].setCANTimeout(250);
          // motors[i].setSmartCurrentLimit(40);
          // motors[i].enableVoltageCompensation(12.0);`
          // motors[i].burnFlash()

          // RelativeEncoder driveEncoder = motors[i].getEncoder();
          // driveEncoder.setPosition(0.0);
          // driveEncoder.setMeasurementPeriod(10);
          // driveEncoder.setAverageDepth(2);

          pidControllers[i] = motors[i].getPIDController();

          // Set PID coefficients
          pidControllers[i].setP(10e-5);
          pidControllers[i].setI(0.0);
          pidControllers[i].setD(0.0);
          pidControllers[i].setFF(0.00008);
          pidControllers[i].setIZone(0.0);
          pidControllers[i].setOutputRange(-1.0, 1.0);
        }

        // pidControllers[0] = motors[0].getPIDController();
        // pidControllers[0].setP(10e-5);
        // pidControllers[0].setI(0.0);
        // pidControllers[0].setD(0.0);
        // pidControllers[0].setFF(0.00008);
        // pidControllers[0].setIZone(0.0);
        // pidControllers[0].setOutputRange(-1.0, 1.0);

        // pidControllers[1] = motors[1].getPIDController();
        // pidControllers[1].setP(10e-5);
        // pidControllers[1].setI(0.0);
        // pidControllers[1].setD(0.0);
        // pidControllers[1].setFF(0.00008);
        // pidControllers[1].setIZone(0.0);
        // pidControllers[1].setOutputRange(-1.0, 1.0);

        // pidControllers[2] = motors[2].getPIDController();
        // pidControllers[2].setP(10e-5);
        // pidControllers[2].setI(0.0);
        // pidControllers[2].setD(0.0);
        // pidControllers[2].setFF(0.00008);
        // pidControllers[2].setIZone(0.0);
        // pidControllers[2].setOutputRange(-1.0, 1.0);

        // motors[1].setInverted(true);
        // motors[2].setInverted(true);

        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Initialize auto chooser
    chooser.addDefaultOption("Default Auto", defaultAuto);
    chooser.addOption("My Auto", customAuto);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.get();
    System.out.println("Auto selected: " + autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
      case customAuto:
        // Put custom auto code here
        break;
      case defaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (linkedShooter.get()) {
      shooterdown.set(shooterup.get());
    }

    if (zero.get()) {
      pidControllers[0].setReference(0, ControlType.kVelocity);
      pidControllers[1].setReference(0, ControlType.kVelocity);
      pidControllers[2].setReference(0, ControlType.kVelocity);
      zero.set(false);
    }

    pidControllers[0].setReference(intake.get(), ControlType.kVelocity);
    pidControllers[1].setReference(shooterup.get(), ControlType.kVelocity);
    pidControllers[2].setReference(shooterdown.get(), ControlType.kVelocity);

    System.out.println("intake " + intake.get());
    System.out.println("up " + shooterup.get());
    System.out.println("down " + shooterdown.get());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
