// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeIndex;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveIntakeWheels extends CommandBase {
  /** Creates a new MoveIntakeWheels. */

  Intake intake;
  double volts;

  public MoveIntakeWheels(Intake intake, double volts) {
    this.intake = intake;
    this.volts = volts;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeMotorVolts(volts);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeMotorVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
