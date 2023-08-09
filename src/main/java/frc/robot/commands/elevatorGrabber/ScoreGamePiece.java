// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.GrabberPlacer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreGamePiece extends SequentialCommandGroup {
  /** Creates a new ScoreGamePiece. */
  public ScoreGamePiece(Elevator elevator, GrabberPlacer grabberPlacer, boolean isCone) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double ejectionVolts = 1;
    if (isCone) { ejectionVolts = -6; }
    
    addCommands(
      // Spits out the game piece.
      new SetGrabberMotor(grabberPlacer, ejectionVolts, 100).withTimeout(0.15),

      // Moves the cube hat (goober) out of the way so the game piece doesn't get pulled out.
      new SetGrabberMotor(grabberPlacer, 6, 100).withTimeout(0.15),
      new InstantCommand(elevator::retractElevator),
      new MoveElevatorToPosition(elevator, 0).asProxy()
    );
  }
}
