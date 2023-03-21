// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.elevatorGrabber.ElevatorIdle;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.commands.intake.MoveIntakeWheels;
import frc.robot.commands.intake.PulseConveyor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Pivot;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexConeFull extends ParallelCommandGroup {
  /** Creates a new IndexConeFull. */
  public IndexConeFull(Intake intake, Indexer indexer, Elevator elevator, Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*
     * TODO: hacky way to allow index cone full to be interrupted by pickupcone
     */
    addRequirements(intake);
    
    addCommands(
    new MoveElevatorToPosition(elevator, 0.4).andThen(new ElevatorIdle(elevator)),
    new SequentialCommandGroup(
      //agitation/alignment procedure
      new InstantCommand(pivot::retractElevator, pivot),
      new MoveClawBack(indexer, -3),
      new ParallelCommandGroup(
        new PulseConveyor(intake, 0.2, 0.05, 3),
        new PulseIndexerWalls(indexer, 0.3),
        new PulseSideBelts(indexer, 0.2, 0.05, 4)
      ).withTimeout(0.6),
      new InstantCommand(indexer::closeIndexerWalls),
      new ParallelCommandGroup(
        new MoveSideBelts(indexer, -5),
        new MoveConveyor(intake, -5)
      ).withTimeout(0.5),


      //uprighting procedure
      new ParallelRaceGroup(
        new MoveConveyor(intake, -3),
        new MoveSideBelts(indexer, -3.6),
        new MoveClawForward(indexer, 3.6).withTimeout(1.0), // at 3.6 before
        new MoveIntakeWheels(intake, 2.0)
      ),
      new ParallelRaceGroup(
        new MoveConveyor(intake, 3),
        new MoveSideBelts(indexer, 3.6),
        new MoveClawBack(indexer, 3.6).withTimeout(1.0),
        new MoveIntakeWheels(intake, 2.0)
      ),

      // NEW STUFF: drivers said to run the claw stuff again
      new ParallelRaceGroup(
        new MoveConveyor(intake, -3),
        new MoveSideBelts(indexer, -3.6),
        new MoveClawForward(indexer, 3.6).withTimeout(1.0),
        new MoveIntakeWheels(intake, 2.0)
      ),
      new ParallelRaceGroup(
        new MoveConveyor(intake, 3),
        new MoveSideBelts(indexer, 3.6),
        new MoveClawBack(indexer, 3.6).withTimeout(1.0),
        new MoveIntakeWheels(intake, 2.0)
      )
    )

      // new MoveElevatorToPosition(elevatorGrabber, 0.13) <- We don't need this, because PickupCone does it already?
    );
  }
}
