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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double controllerX = drivetrain.modifyAxis(-RobotContainer.controller.getLeftX());
    double controllerY = drivetrain.modifyAxis(-RobotContainer.controller.getLeftY());
    double controllerR = drivetrain.modifyAxis(-RobotContainer.controller.getRightX());

    //raw controller values after modifyAxis will be between -1 and 1
    //coefficient = maximum speed in meters or radians per second

    ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds(
      controllerY*Constants.Swerve.maxVelocityMetersPerSecond,
      controllerX*Constants.Swerve.maxVelocityMetersPerSecond,
      controllerR*Constants.Swerve.maxAngularVelocityRadiansPerSecond
    );

    if (fieldOriented) {
      outputChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        outputChassisSpeeds,
        drivetrain.getGyroscopeRotation());
    }

    drivetrain.drive(
      outputChassisSpeeds
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }
}