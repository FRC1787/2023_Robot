// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intakeIndex.MoveConveyor;
import frc.robot.commands.intakeIndex.MoveIntakeWheels;
import frc.robot.commands.intakeIndex.PulseConveyor;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexConeFull extends SequentialCommandGroup {
  /** Creates a new IndexConeFull. */
  public IndexConeFull(Intake intake, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(
      new ParallelCommandGroup(
        new PulseConveyor(intake, 0.4, .15),
        new PulseIndexerWalls(indexer, 0.5),
        new PulseSideBelts(indexer, 0.4, .1)
      ).withTimeout(1),
      new InstantCommand(indexer::closeIndexerWalls),
      new ParallelCommandGroup(
        new MoveSideBelts(indexer, -0.3),
        new MoveConveyor(intake, -0.3)
      ).withTimeout(1),
      new InstantCommand(indexer::closeIndexerWalls),
      new ParallelRaceGroup(
        new MoveConveyor(intake, -0.25),
        new MoveSideBelts(indexer, -0.3),
        new MoveClawForward(indexer, 0.3),
        new MoveIntakeWheels(intake, 2.0)
      ),
      new ParallelRaceGroup(
        new MoveConveyor(intake, 0.25),
        new MoveSideBelts(indexer, 0.3),
        new MoveClawBack(indexer, 0.3),
        new MoveIntakeWheels(intake, 2.0)
        
      )

    );
  }
}
