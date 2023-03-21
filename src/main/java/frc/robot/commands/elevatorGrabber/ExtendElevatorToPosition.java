// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendElevatorToPosition extends SequentialCommandGroup {
  /** Creates a new ExtendElevatorToPosition. */
  public ExtendElevatorToPosition(Elevator elevator, Pivot pivot, double elevatorPositionMeters) {
    addCommands(
      new InstantCommand(pivot::extendElevator, pivot),
      new WaitCommand(0.2),
      new MoveElevatorToPosition(elevator, elevatorPositionMeters).asProxy()
    );
  }
}
