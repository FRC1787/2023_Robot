// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AlignWheelsToZero extends CommandBase {
  private Drivetrain drivetrain;
  private SwerveModuleState[] desiredStates = {
    new SwerveModuleState(0, new Rotation2d(0)),
    new SwerveModuleState(0, new Rotation2d(0)),
    new SwerveModuleState(0, new Rotation2d(0)),
    new SwerveModuleState(0, new Rotation2d(0))
  };;

  /** Creates a new AlignWheelsToZero. */
  public AlignWheelsToZero(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setModuleStatesClosedLoopNoOptimize(desiredStates);
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
