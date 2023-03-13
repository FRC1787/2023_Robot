// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.MoveSideBelts;
import frc.robot.commands.intake.MoveConveyor;
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
      // new MoveElevatorToPosition(elevatorGrabber, 0.27),
      new InstantCommand(indexer::closeIndexerWalls),
      // new ParallelCommandGroup(
      //   new MoveSideBelts(indexer, 1.2).withTimeout(1.0),
      //   new MoveConveyor(intake, 0.25*12).withTimeout(1.0)
      // ),
      new MoveElevatorToPosition(elevatorGrabber, 0.0),
      new ParallelCommandGroup(
        new MoveSideBelts(indexer, -2.0),
        new MoveConveyor(intake, -4)
      ).withTimeout(0.75),
      new ParallelRaceGroup(
        new SetGrabberMotor(elevatorGrabber, -6, 14).withTimeout(1.50),
        new MoveSideBelts(indexer, -2.0),
        new MoveConveyor(intake, -4)  
      ),
      new InstantCommand(indexer::openIndexerWalls),
      new ParallelCommandGroup(
        new SetGrabberMotor(elevatorGrabber, -6, 14).withTimeout(1.5)
      ),
      new SetGrabberMotor(elevatorGrabber, -0.5, 100)
    );
  }
}
