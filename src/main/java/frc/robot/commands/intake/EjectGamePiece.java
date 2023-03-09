// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class EjectGamePiece extends CommandBase {
  private Intake intake;
  private Indexer indexer;
  private double intakeMotorVoltage;
  private double conveyorMotorVoltage;

  /**
   * Extends the intake, closes indexer walls, and ejects a game piece.
   * @param intake - intake subsystem object
   * @param indexer - indexer subsystem object
   * @param intakeMotorVoltage - Voltage to send to the intake (upper) motor. Make this positive to eject a piece.
   * @param conveyorMotorVoltage - Voltage to send to the conveyor (lower) motor. Make this positive to eject a piece.
   */
  public EjectGamePiece(Intake intake, Indexer indexer, double intakeMotorVoltage, double conveyorMotorVoltage) {
    addRequirements(intake);
    this.intake = intake;
    this.indexer = indexer;
    this.intakeMotorVoltage = intakeMotorVoltage;
    this.conveyorMotorVoltage = conveyorMotorVoltage;

    // SmartDashboard.putNumber("intake motor voltage", 0.0);
    // SmartDashboard.putNumber("conveyor motor voltage", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // intakeMotorVoltage = SmartDashboard.getNumber("intake motor voltage", 0.0);
    // conveyorMotorVoltage = SmartDashboard.getNumber("conveyor motor voltage", 0.0);

    intake.extendIntake();
    intake.setIntakeMotorVolts(intakeMotorVoltage);
    intake.setConveyorMotorVolts(conveyorMotorVoltage);
    indexer.closeIndexerWalls();
    indexer.setIndexerMotors(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeMotors();
    intake.retractIntake();
    indexer.setIndexerMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
