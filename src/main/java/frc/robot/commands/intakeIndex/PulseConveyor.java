// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeIndex;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class PulseConveyor extends CommandBase {
  /** Creates a new PulseConveyor. */
  private Intake intake;
  private Timer timer;
  private double timeBetweenPulses;
  private double volts;
  
  /**
   * Pulses conveyor forwards and backwards
   * @param intake - intake subsystem
   * @param time - time between each pulse of the conveyor
   * @param volts - should ALWAYS be positive, volts to apply to the conveyor motor
   */
  public PulseConveyor(Intake intake, double time, double volts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.volts = volts;
    timer = new Timer();
    timeBetweenPulses = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if(intake.getConveyorMotorDirection() < 0) {
      intake.setConveyorMotorVolts(volts);
    }else {
      intake.setConveyorMotorVolts(-volts);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= timeBetweenPulses) {
      if(intake.getConveyorMotorDirection() < 0) {
        intake.setConveyorMotorVolts(volts);
      }else {
        intake.setConveyorMotorVolts(-volts);
      }
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setConveyorMotorVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
