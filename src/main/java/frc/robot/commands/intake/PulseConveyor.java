// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeIndex.Conveyor;

public class PulseConveyor extends CommandBase {
  /** Creates a new PulseConveyor. */
  private Conveyor conveyor;
  private Timer timer;
  private double timeBetweenPulsesBack;
  private double timeBetweenPulsesForward;
  private double volts;
  
  /**
   * Pulses conveyor forwards and backwards
   * @param conveyor - intake subsystem
   * @param time - time between each pulse of the conveyor
   * @param volts - should ALWAYS be positive, volts to apply to the conveyor motor
   */
  public PulseConveyor(Conveyor conveyor, double timeBack, double timeForward, double volts) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);

    this.conveyor = conveyor;
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
    if(conveyor.getConveyorMotorDirection() < 0) {
      conveyor.setConveyorMotorVolts(volts);
    }else {
      conveyor.setConveyorMotorVolts(-volts);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(conveyor.getConveyorMotorDirection() < 0) {
      if(timer.get() >= timeBetweenPulsesBack) {
        conveyor.setConveyorMotorVolts(volts);
        timer.reset();
      }
    }else {
      if(timer.get() >= timeBetweenPulsesForward) {
        conveyor.setConveyorMotorVolts(-volts);
        timer.reset();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.setConveyorMotorVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
