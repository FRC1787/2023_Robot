// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeIndex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeGamePieces extends CommandBase {
  private Intake intakeIndex;
  private double intakeMotorVoltage;
  private double conveyorMotorVoltage;

  public IntakeGamePieces(Intake intakeIndex, double intakeMotorVoltage, double conveyorMotorVoltage) {
    addRequirements(intakeIndex);
    this.intakeIndex = intakeIndex;
    this.intakeMotorVoltage = Math.abs(intakeMotorVoltage);
    this.conveyorMotorVoltage = Math.abs(conveyorMotorVoltage);

    SmartDashboard.putNumber("intake motor voltage", 0.0);
    SmartDashboard.putNumber("conveyor motor voltage", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeMotorVoltage = SmartDashboard.getNumber("intake motor voltage", 0.0);
    conveyorMotorVoltage = SmartDashboard.getNumber("conveyor motor voltage", 0.0);

    intakeIndex.extendIntake();
    intakeIndex.setIntakeMotorVolts(intakeMotorVoltage);
    intakeIndex.setConveyorMotorVolts(conveyorMotorVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeIndex.stopIntakeMotors();
    intakeIndex.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
