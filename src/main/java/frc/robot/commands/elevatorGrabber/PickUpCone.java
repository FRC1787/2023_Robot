// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intakeIndex.MoveConveyor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpCone extends SequentialCommandGroup {
  /** Creates a new PickUpCone. */
  public PickUpCone(ElevatorGrabber elevatorGrabber, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new MoveConveyor(intake, -0.1),
        new SequentialCommandGroup( //0.26017 is for during indexing
          new MoveElevatorToPosition(elevatorGrabber, 0.27),
          new InstantCommand(elevatorGrabber::extendElevator),
          new ParallelCommandGroup(
            new SetGrabberMotor(elevatorGrabber, 9, 17),
            new MoveElevatorToPosition(elevatorGrabber, 0.04)
          )  
        )
      )
    );
  }
}
