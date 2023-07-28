// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class RotateToAngle extends CommandBase {
  double angle;
  Drivetrain drivetrain;
  PIDController angleController = new PIDController(8, 0, 0.3);

  /** Rotates to a specific field-oriented angle. Set to 180 for rotating to face the grid. */
  public RotateToAngle(double angle, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.angle = angle;
    this.drivetrain = drivetrain;

    angleController.setTolerance(1, 0.5);
    angleController.setSetpoint(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      new ChassisSpeeds(
        0, 0, MathUtil.clamp(
          angleController.calculate(drivetrain.getRobotRotation2d().getDegrees() % 360),
          -270,
          270
        )
      ),
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(
      new ChassisSpeeds(0, 0, 0),
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleController.atSetpoint();
  }
}
