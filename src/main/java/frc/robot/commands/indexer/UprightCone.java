// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.commands.intake.MoveIntakeWheels;
import frc.robot.subsystems.intakeIndex.Intake;
import frc.robot.subsystems.intakeIndex.Claw;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.intakeIndex.IndexerWalls;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UprightCone extends SequentialCommandGroup {
  /** Creates a new UprightCone. */
  public UprightCone(Intake intake, Conveyor conveyor, IndexerWalls indexerWalls, Claw claw, double clawDistance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ParallelRaceGroup(
        new MoveConveyor(conveyor, -3),
        new MoveSideBelts(indexerWalls, -3.6),
        new MoveClawForward(claw, 3.6, clawDistance).withTimeout(1.0), // volts was at 3.6 before
        new MoveIntakeWheels(intake, 2.0)
      ),
      new ParallelRaceGroup(
        new MoveConveyor(conveyor, 3),
        new MoveSideBelts(indexerWalls, 3.6),
        new MoveClawBack(claw, 3.6).withTimeout(1.0),
        new MoveIntakeWheels(intake, 2.0)
      )
    );
  }
}
