// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorGrabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractAndHomeElevator extends ParallelCommandGroup {
  /** Creates a new RetractAndHomeElevator. */
  public RetractAndHomeElevator(ElevatorGrabber elevatorGrabber) {
    addCommands(
      /*
       * TODO: Make sure the elevator has retracted a decent amount
       *       before retracting the pneumatics to ensure we don't
       *       break the vertical height limit.
       *       We could do this simply with some delay, but I'm also wondering
       *       if we can have pneumatic retraction trigger when the elevator
       *       reaches a certain height (i.e. short enough to not break the height
       *       limit when retracting).
       */
      new InstantCommand(elevatorGrabber::retractElevator),
      new MoveElevatorToPosition(elevatorGrabber, 0)
    );
  }
}
