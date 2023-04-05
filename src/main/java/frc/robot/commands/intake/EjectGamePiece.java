// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.GrabberPlacer;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.intakeIndex.IndexerWalls;
import frc.robot.subsystems.intakeIndex.Intake;

public class EjectGamePiece extends SequentialCommandGroup {
  /**
   * Extends the intake, closes indexer walls, and ejects a game piece.
   * @param intake - intake subsystem object
   * @param indexer - indexer subsystem object
   * @param elevatorGrabber - elevatorGrabber subsystem object
   * @param grabberPlacer - grabberPlacer subsystem object
   * @param intakeMotorVoltage - Voltage to send to the intake (upper) motor. Make this positive to eject a piece.
   * @param conveyorMotorVoltage - Voltage to send to the conveyor (lower) motor. Make this positive to eject a piece.
   * @param indexerMotorVoltage - Voltage to send to the side belts. Make this positive to eject a piece.
   * @param grabberPlacerVolts - Voltage to send to grabber place. Make this positive to eject a piece.
   */
  public EjectGamePiece(Intake intake, Pivot pivot, Conveyor conveyor, IndexerWalls indexerWalls, GrabberPlacer grabberPlacer, double intakeMotorVoltage, double conveyorMotorVoltage, double indexerMotorVoltage, double grabberPlacerVolts) {
    addCommands(
      new InstantCommand(pivot::retractElevator),
      new InstantCommand(intake::extendIntake),
      new BowlCube(intake, conveyor, indexerWalls, grabberPlacer, intakeMotorVoltage, conveyorMotorVoltage, indexerMotorVoltage, grabberPlacerVolts)
    );
  }
}