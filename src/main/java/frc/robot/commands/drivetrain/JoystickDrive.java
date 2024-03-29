// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;


public class JoystickDrive extends CommandBase {
  private Drivetrain drivetrain;
  private boolean fieldOriented;

  /**
   * Command that reads from controller input and moves robot in that way
   * Set as default command
   */
  public JoystickDrive(Drivetrain drivetrain, boolean fieldOriented) {
    addRequirements(drivetrain);
    this.drivetrain=drivetrain;
    this.fieldOriented=fieldOriented;
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double controllerX = modifyAxis(-RobotContainer.controller.getLeftX());
    double controllerY = modifyAxis(-RobotContainer.controller.getLeftY());
    double controllerR = modifyAxis(-RobotContainer.controller.getRightX());

    // Raw controller values after modifyAxis will be between -1 and 1.
    // Coefficient = maximum speed in meters or radians per second.

    ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds(
      controllerY*Constants.Swerve.maxDesiredTeleopVelocityMetersPerSecond,
      controllerX*Constants.Swerve.maxDesiredTeleopVelocityMetersPerSecond,
      controllerR*Constants.Swerve.maxDesiredAngularVelocityRadiansPerSecond
    );

    if (fieldOriented) {
      outputChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        outputChassisSpeeds,
        drivetrain.getRobotRotation2d());
    }

    drivetrain.drive(outputChassisSpeeds, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0), true);
  }
}