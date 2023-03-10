// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpCone extends SequentialCommandGroup {
  /** Creates a new PickUpCone. */
  public PickUpCone(ElevatorGrabber elevatorGrabber, Intake intake, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double grabbingVolts = 6;
    double pickupPosition = 0.14;
    double grabbingAmpLimit = 17;
    addCommands(
      new InstantCommand(indexer::closeIndexerWalls),
      new InstantCommand(elevatorGrabber::retractElevator),
      new MoveElevatorToPosition(elevatorGrabber, pickupPosition),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new MoveConveyor(intake, -6).withTimeout(0.5),
          new MoveConveyor(intake, 4)
        ),
        new SetGrabberMotor(elevatorGrabber, grabbingVolts, grabbingAmpLimit).withTimeout(1.0)
      )
    );

    //0.09
  }
}
