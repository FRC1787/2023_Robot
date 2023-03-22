// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.MoveClawBack;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.elevator.GrabberPlacer;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Claw;
import frc.robot.subsystems.intakeIndex.IndexerWalls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpCone extends SequentialCommandGroup {
  /** Creates a new PickUpCone. */
  public PickUpCone(Elevator elevator, Pivot pivot, GrabberPlacer grabberPlacer, Intake intake, IndexerWalls indexerWalls, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(intake); //TODO: intake
    
    double grabbingVolts = 6;
    double pickupPosition = 0.135;
    double grabbingAmpLimit = 26;
    addCommands(
      // move all subsystems into the grabbing position
      new InstantCommand(indexerWalls::closeIndexerWalls),
      new InstantCommand(pivot::retractElevator, pivot),
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, pickupPosition).asProxy(),
        new MoveClawBack(claw, -3)
      ),

      // start spinning the grabber wheel, while the belts push the cone
      // into the grabber wheel, then reverse the belts to kick up the cone.
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new MoveConveyor(intake, -10).withTimeout(0.35), // 0.525
          new MoveConveyor(intake, 3.5)
        ),
        new SetGrabberMotor(grabberPlacer, grabbingVolts, grabbingAmpLimit).withTimeout(2)
      )
    );
  }
}
