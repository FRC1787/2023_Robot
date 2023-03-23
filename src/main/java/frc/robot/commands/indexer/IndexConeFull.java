// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.intake.PulseConveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Claw;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.intakeIndex.IndexerWalls;
import frc.robot.subsystems.intakeIndex.Intake;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexConeFull extends SequentialCommandGroup {
  /** Creates a new IndexConeFull. */
  public IndexConeFull(Intake intake, Conveyor conveyor, IndexerWalls indexerWalls, Claw claw, Elevator elevator, Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      //agitation/alignment procedure
      new InstantCommand(pivot::retractElevator, pivot),
      new MoveClawBack(claw, -3),
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, 0.4).asProxy(), // https://www.chiefdelphi.com/t/sequential-command-group-default-command-issue/430845
        new PulseConveyor(conveyor, 0.2, 0.05, 3),
        new PulseSideBelts(indexerWalls, 0.2, 0.05, 4),
        new SequentialCommandGroup(
          new InstantCommand(indexerWalls::closeIndexerWalls)
            .andThen(new WaitCommand(.3)),
          new InstantCommand(indexerWalls::openIndexerWalls)
            .andThen(new WaitCommand(.3))
        ).repeatedly()
      ).withTimeout(0.6),
      new InstantCommand(indexerWalls::closeIndexerWalls),
      // new ParallelCommandGroup(
      //   new MoveSideBelts(indexerWalls, -5),
      //   new MoveConveyor(intake, -5)
      // ).withTimeout(0.5),

      //uprighting procedure

      new UprightCone(intake, conveyor, indexerWalls, claw),
      new UprightCone(intake, conveyor, indexerWalls, claw),
      new UprightCone(intake, conveyor, indexerWalls, claw),
      new UprightCone(intake, conveyor, indexerWalls, claw)
      // new MoveElevatorToPosition(elevatorGrabber, 0.13) <- We don't need this, because PickupCone does it already?
    );
  }
}