// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.intakeIndex.IndexerWalls;
import frc.robot.subsystems.intakeIndex.Intake;

public class IntakeGamePieces extends CommandBase {
  private Intake intake;
  private Conveyor conveyor;
  private IndexerWalls indexerWalls;
  private Pivot pivot;
  private double intakeMotorVoltage;
  private double conveyorMotorVoltage;
  private double sideBeltVoltage;

  /**
   * Extends the intake and intakes a game piece.
   * @param intake - intake subsystem object
   * @param intakeMotorVoltage - Voltage to send to the intake (upper) motor. Make this negative to intake a piece.
   * @param conveyorMotorVoltage - Voltage to send to the conveyor (lower) motor. Make this negative to intake a piece.
   * @param sideBeltVoltage - Voltage to send to the side belt motors. Make this negative to intake a piece.
   */
  public IntakeGamePieces(Intake intake, Conveyor conveyor, IndexerWalls indexerWalls, Pivot pivot, double intakeMotorVoltage, double conveyorMotorVoltage, double sideBeltVoltage) {
    addRequirements(intake, conveyor, indexerWalls, pivot);
    this.intake = intake;
    this.conveyor = conveyor;
    this.indexerWalls = indexerWalls;
    this.pivot = pivot;
    this.intakeMotorVoltage = intakeMotorVoltage;
    this.conveyorMotorVoltage = conveyorMotorVoltage;
    this.sideBeltVoltage = sideBeltVoltage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.extendIntake();
    intake.setIntakeMotorVolts(intakeMotorVoltage);
    conveyor.setConveyorMotorVolts(conveyorMotorVoltage);
    pivot.retractElevator();
    indexerWalls.openIndexerWalls();
    indexerWalls.setIndexerMotors(sideBeltVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeMotor();
    intake.retractIntake();
    conveyor.stopIntakeMotors();
    indexerWalls.setIndexerMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
