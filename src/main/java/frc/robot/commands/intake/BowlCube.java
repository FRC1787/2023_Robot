// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.MoveSideBelts;
import frc.robot.subsystems.elevator.GrabberPlacer;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.intakeIndex.IndexerWalls;
import frc.robot.subsystems.intakeIndex.Intake;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;

public class BowlCube extends SequentialCommandGroup {
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
  public BowlCube(Intake intake, Conveyor conveyor, IndexerWalls indexerWalls, GrabberPlacer grabberPlacer, double intakeMotorVoltage, double conveyorMotorVoltage, double indexerMotorVoltage, double grabberPlacerVolts) {
    addCommands(
      new SetGrabberMotor(grabberPlacer, 12, 100).withTimeout(0.15),
      new InstantCommand(indexerWalls::closeIndexerWalls),
      new ParallelCommandGroup(
        new MoveConveyor(conveyor, conveyorMotorVoltage),
        new MoveSideBelts(indexerWalls, indexerMotorVoltage),
        new MoveIntakeWheels(intake, intakeMotorVoltage)
      )
    );
  }
}