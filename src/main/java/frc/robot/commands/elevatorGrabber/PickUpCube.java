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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.GrabberPlacer;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.intakeIndex.IndexerWalls;
import frc.robot.subsystems.intakeIndex.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpCube extends SequentialCommandGroup {
  /** Creates a new PickUpCube. */
  public PickUpCube(Intake intake, Conveyor conveyor, Elevator elevator, Pivot pivot, GrabberPlacer grabberPlacer, IndexerWalls indexerWalls) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Moves subsystems into the pickup position.
      new InstantCommand(pivot::retractElevator, pivot),
      new InstantCommand(indexerWalls::closeIndexerWalls),
      new MoveElevatorToPosition(elevator, 0.0).asProxy(),

      // start pushing the cube towards the grabber wheels,
      // then spin the grabber wheels while the cube is being pushed into them
      // to pickup the cube.

      // Pushes the cube towards the Grabber Placer wheels which will pick up the cube on contact with it's spinning wheels.
      new ParallelRaceGroup(
        new SetGrabberMotor(grabberPlacer, -6, 14).withTimeout(1.50),
        new MoveSideBelts(indexerWalls, -2.0),
        new MoveConveyor(conveyor, -4)  
      ),

      // TODO: i dont even know whats going on here lol
      
      // Now that the cube is securly held by the Grabber Placer, we can open the indexer,
      // and also have the grabber apply a small torque to hold onto the cube.
      new InstantCommand(indexerWalls::openIndexerWalls),
      new ParallelCommandGroup(
        new SetGrabberMotor(grabberPlacer, -6, 14).withTimeout(1.5)
      ),
      new SetGrabberMotor(grabberPlacer, -0.5, 100)
    );
  }
}
