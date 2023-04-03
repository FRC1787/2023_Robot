// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.MoveClawBack;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.GrabberPlacer;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Claw;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.intakeIndex.IndexerWalls;
import frc.robot.subsystems.intakeIndex.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpCone extends SequentialCommandGroup {
  /** Creates a new PickUpCone. */
  public PickUpCone(Elevator elevator, Pivot pivot, GrabberPlacer grabberPlacer, Intake intake, Conveyor conveyor, IndexerWalls indexerWalls, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    double grabbingVolts = 6;
    double pickupPosition = 0.135;
    double grabbingAmpLimit = 32; //26;
    addCommands(
      // move all subsystems into the grabbing position
      new InstantCommand(indexerWalls::closeIndexerWalls, indexerWalls),
      new InstantCommand(pivot::retractElevator, pivot),
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, pickupPosition).asProxy(),
        new MoveClawBack(claw, -3)
      ),

      // start spinning the grabber wheel, while the belts push the cone
      // into the grabber wheel, then reverse the belts to kick up the cone.
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new MoveConveyor(conveyor, -12).withTimeout(0.25),
          new MoveConveyor(conveyor, 3).withTimeout(0.5)
        ),
        new SetGrabberMotor(grabberPlacer, grabbingVolts, grabbingAmpLimit)
      ).repeatedly().until(() -> {return grabberPlacer.getGrabOutputAmps() >= grabbingAmpLimit;})
/*
      new MoveConveyor(conveyor, 12).withTimeout(0.25),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          // In the best case, we grab the cone after the first time the
          // conveyor kicks it forward. However there are still some instances
          // where the cone starts far away from the grabber and doesn't make
          // contact with the grabber in time for the kick by the conveyor.
          // Increasing the back drive time to account for this will slow down our most
          // common case where the cone is already pretty close to the grabber, so
          // we just decided to just automatically re-try the whole sequence
          // sequence 3 times for if it doesn't work the first time.
          new MoveConveyor(conveyor, -12).withTimeout(0.25),
          new MoveConveyor(conveyor, 3).withTimeout(0.25),
          new MoveConveyor(conveyor, -12).withTimeout(0.25),
          new MoveConveyor(conveyor, 3).withTimeout(0.25),
          new MoveConveyor(conveyor, -12).withTimeout(0.25),
          new MoveConveyor(conveyor, 3).withTimeout(0.25)
        ),
        new SetGrabberMotor(grabberPlacer, grabbingVolts, grabbingAmpLimit)
      )
*/
    );
  }
}
