// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intakeIndex.MoveConveyor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpCone extends SequentialCommandGroup {
  /** Creates a new PickUpCone. */
  public PickUpCone(ElevatorGrabber elevatorGrabber, Intake intake, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double grabbingVolts = 6;
    double holdingVolts = 1.0;
    double pickupPosition = 0.10;
    double grabbingAmpLimit = 25;
    double holdingAmpLimit = 50;
    addCommands(
      new InstantCommand(indexer::closeIndexerWalls),
      new InstantCommand(elevatorGrabber::retractElevator),
      new MoveElevatorToPosition(elevatorGrabber, pickupPosition),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new MoveConveyor(intake, -0.5).withTimeout(1),
          new MoveConveyor(intake, 0.1)
        ),
        new SetGrabberMotor(elevatorGrabber, grabbingVolts, grabbingAmpLimit)
      ),
      new ParallelRaceGroup(
        new MoveElevatorToPosition(elevatorGrabber, 1.0),
        new SetGrabberMotor(elevatorGrabber, holdingVolts, holdingAmpLimit)
      )
    );
  }
}
