// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LoggerMode;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  //turn this true to use the simulator!!
  public LoggerMode robotMode = LoggerMode.ROBOT_REAL_LOGGED;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private String previousPath = "";


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    if (isReal()) Timer.delay(10); // Ghost Busters! Make sure everything has time to power up before initializing
    
    Logger.getInstance().recordMetadata("projectName", "2023Robot");

    switch (robotMode) {
      case ROBOT_SIM:
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        break;
      case ROBOT_REAL:
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        break;
      case ROBOT_REAL_LOGGED:
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser/deploy/logs")); 
        Logger.getInstance().addDataReceiver(new NT4Publisher());
        break;
      case ROBOT_REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
        break;
    }
    
    Logger.getInstance().start();
    
    
    m_robotContainer = new RobotContainer();

    PathPlannerServer.startServer(5811);

    //m_robotContainer.drivetrain.zeroYaw();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
  public void disabledPeriodic() {
    String path = m_robotContainer.autoChooser.getSelected();
    if (previousPath.equals(path)) return;

    m_robotContainer.pathGroup = PathPlanner.loadPathGroup(
      path, PathPlanner.getConstraintsFromPath(path));
    previousPath = path;
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    m_robotContainer.elevator.retractElevator();
      
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().schedule(new MoveElevatorToPosition(m_robotContainer.elevator, 0));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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
