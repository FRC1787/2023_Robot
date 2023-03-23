// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeIndex.IndexerWalls;

public class PulseIndexerWalls extends CommandBase {
  private IndexerWalls indexerWalls;
  double time;
  Timer timer;
  /** Creates a new PulseIndexerWalls. */
  public PulseIndexerWalls(IndexerWalls indexerWalls, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexerWalls);

    this.indexerWalls = indexerWalls;
    this.time = time;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    if(indexerWalls.isIndexerWallsOpen()) {
      indexerWalls.closeIndexerWalls();
    } else {
      indexerWalls.openIndexerWalls();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= time) {
      if(indexerWalls.isIndexerWallsOpen()) {
        indexerWalls.closeIndexerWalls();
      } else {
        indexerWalls.openIndexerWalls();
      }
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
