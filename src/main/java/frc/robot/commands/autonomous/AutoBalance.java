// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;



public class AutoBalance extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final PIDController tiltController;
  private boolean hasTilted;

  public AutoBalance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // TODO: tune pid controller and maybe add rotation adjustment if needed
    tiltController = new PIDController(0.1/10, 0, 0); 
    tiltController.setSetpoint(0);

    hasTilted = false;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(1.5, 0., 0., drivetrain.getRobotRotation2d()), true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(drivetrain.getRobotPitchDegreesPerSecond()) > 50) {
      hasTilted = true;
    }
    if (hasTilted) {
      double desiredVxMetersPerSecond = tiltController.calculate(drivetrain.getRobotPitchDegrees());
      drivetrain.drive(new ChassisSpeeds(desiredVxMetersPerSecond, 0, 0), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0), true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
