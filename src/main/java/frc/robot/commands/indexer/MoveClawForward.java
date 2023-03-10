// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class MoveClawForward extends CommandBase {
  /**
   * Moves claw towards back of robot.
   * @param percentage - percentage/speed of claw motor, the sign does not matter.
   */
  private Indexer indexer;
  private double clawMotorVoltage;
  private Timer timer;

  public MoveClawForward(Indexer indexer, double clawMotorVoltage) {
    this.indexer = indexer;
    this.clawMotorVoltage = Math.abs(clawMotorVoltage);

    timer = new Timer();
    
    // SmartDashboard.putNumber("indexer claw forward voltage", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

    // clawMotorVoltage = SmartDashboard.getNumber("indexer claw forward voltage", 0.0);
    timer.start();
    indexer.setClawMotorVolts(clawMotorVoltage);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setClawMotorVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.isClawForward();
  }
}
