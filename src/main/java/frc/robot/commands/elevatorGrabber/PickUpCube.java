// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakeIndex.MoveConveyor;
import frc.robot.commands.intakeIndex.MoveSideBelts;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpCube extends SequentialCommandGroup {
  /** Creates a new PickUpCube. */
  public PickUpCube(Intake intake, ElevatorGrabber elevatorGrabber, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(elevatorGrabber::retractElevator),
      new MoveElevatorToPosition(elevatorGrabber, 0.27),
      new InstantCommand(indexer::closeIndexerWalls),
      new ParallelCommandGroup(
        new MoveSideBelts(indexer, 0.1).withTimeout(1.0),
        new MoveConveyor(intake, 0.25).withTimeout(1.0)
      ),
      new MoveElevatorToPosition(elevatorGrabber, -0.1),
      new ParallelRaceGroup(
        new MoveSideBelts(indexer, -0.1).withTimeout(3.0),
        new MoveConveyor(intake, -0.25).withTimeout(3.0),
        new SetGrabberMotor(elevatorGrabber, -6, 8)
      ),
      new InstantCommand(indexer::openIndexerWalls),
      new ParallelCommandGroup(
        new SetGrabberMotor(elevatorGrabber, -6, 8)
      ),
      new MoveElevatorToPosition(elevatorGrabber, 0.3)
    );
  }
}
