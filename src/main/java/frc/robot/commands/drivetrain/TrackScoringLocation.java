// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class TrackScoringLocation extends CommandBase {
  

  Drivetrain drivetrain;
  Elevator elevator;

  PIDController anglePID = new PIDController(8, 0, 0);  //(8, 0, 0.3)
  double targetX = 0.78;
  double targetY = 2.73;


  public TrackScoringLocation(Drivetrain drivetrain, Elevator elevator) {
    this.drivetrain=drivetrain;
    this.elevator=elevator;
    addRequirements(drivetrain, elevator);
    
    anglePID.enableContinuousInput(-180, 180);
  }


  /**
   * Returns 0 if the parameter {@code num} is lower than {@code deadzone} to
   * prevent joystick drift.
   * 
   * @param num      Axis input value
   * @param deadzone Lowest value before input is set to 0
   * @return Axis input checked against deadzone value
   */
  private double deadzone(double num, double deadzone) {
    return Math.abs(num) > deadzone ? num : 0;
  }

  /**
   * Adds a deadzone to axis input and squares the input.
   * 
   * This function should always return a value between -1 and 1.
   * 
   * @param value Axis input
   * @return Squared and deadzoned input
   */
  private double modifyAxis(double value) {
    value = deadzone(value, Constants.Controller.controllerDeadzone);

    //adjust this power value for diffferences in how the robot handles (recommended between 1.5 and 3)
    return Math.signum(value) * Math.pow(Math.abs(value), 2.3);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //drivetrain stuff ////////////////////////////////////////////////////////////////
    double controllerX = modifyAxis(-RobotContainer.controller.getLeftX());
    double controllerY = modifyAxis(-RobotContainer.controller.getLeftY());

    Pose2d robotPose = drivetrain.getEstimatorPoseMeters();
    double deltaX = robotPose.getX()-targetX;
    double deltaY = robotPose.getY()-targetY;

    double atanAngleDeg = Math.toDegrees(Math.atan(deltaX/deltaY));
    double targetAngleDeg = Math.signum(atanAngleDeg)*(-90) - atanAngleDeg; //this is definitely not the simplest way of doing this but it works
    
    double robotAngleDeg = robotPose.getRotation().getDegrees();

    SmartDashboard.putNumber("target angle deg", targetAngleDeg);
    SmartDashboard.putNumber("robot angle deg", robotAngleDeg);

    double pidOutputDegreesPerSecond = anglePID.calculate(robotAngleDeg, targetAngleDeg);

    ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds(
      controllerY*2, //2, since max speed in meters per second while doing chicken head is 2 m/s (slower than default of 4.3)
      controllerX*2,
      MathUtil.clamp(Math.toRadians(pidOutputDegreesPerSecond), -3, 3) //clamp for safety reasons
    );

    outputChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      outputChassisSpeeds,
      drivetrain.getRobotRotation2d());

    drivetrain.drive(outputChassisSpeeds, true);

    //elevator position stuff ////////////////////////////////////////////////////////////


    double distToTargetMeters = Math.sqrt(deltaX*deltaX+deltaY*deltaY);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
