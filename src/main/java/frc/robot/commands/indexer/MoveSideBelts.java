// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class MoveSideBelts extends CommandBase {
  

  Indexer indexer;
  double indexerSideBeltsMotorVoltage;

  /**
   * Moves side belts at a given voltage.
   * @param indexer - indexer subsystem object
   * @param indexerSideBeltsMotorVoltage - a positive value will move the belts towards the front of the robot.
   */
  public MoveSideBelts(Indexer indexer, double indexerSideBeltsMotorVoltage) {
    // addRequirements(indexer);

    this.indexer = indexer;
    this.indexerSideBeltsMotorVoltage = indexerSideBeltsMotorVoltage;

    
    // SmartDashboard.putNumber("indexer side belt volts", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // indexerSideBeltsMotorVoltage = SmartDashboard.getNumber("indexer side belt volts", 0.0);

    indexer.setIndexerMotors(indexerSideBeltsMotorVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setIndexerMotors(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
