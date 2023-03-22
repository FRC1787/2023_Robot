// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeIndex.IndexerWalls;

public class PulseSideBelts extends CommandBase {
  private IndexerWalls indexerWalls;
  private Timer timer;
  private double timeBetweenPulsesBack;
  private double timeBetweenPulsesForward;
  private double volts;
  
  /**
   * Pulses side belts forward and backward
   * @param indexer - indexer subsystem
   * @param time - time between each pulse
   * @param volts - should ALWAYS be positive, volts to apply to indexer motors
   */
  public PulseSideBelts(IndexerWalls indexerWalls, double timeBack, double timeForward, double volts) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexerWalls);
    this.indexerWalls = indexerWalls;
    this.volts = volts;
    timer = new Timer();
    timeBetweenPulsesBack = timeBack;
    timeBetweenPulsesForward = timeForward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if(indexerWalls.getIndexerDirection() < 0) {
      indexerWalls.setIndexerMotors(volts);
    }else {
      indexerWalls.setIndexerMotors(-volts);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexerWalls.getIndexerDirection() < 0) {
      if(timer.get() >= timeBetweenPulsesBack) {
        indexerWalls.setIndexerMotors(volts);
        timer.reset();
      }
    }else {
      if(timer.get() >= timeBetweenPulsesForward) {
        indexerWalls.setIndexerMotors(-volts);
        timer.reset();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerWalls.setIndexerMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
