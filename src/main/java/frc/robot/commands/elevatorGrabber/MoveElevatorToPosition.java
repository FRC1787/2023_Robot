// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorGrabber;

public class MoveElevatorToPosition extends CommandBase {

    ElevatorGrabber elevatorGrabber;
    double targetPositionMeters;
    TrapezoidProfile profile;
    Timer timer;

    /**
     * 
     * @param elevatorGrabber
     * @param targetPositionMeters
     */
    public MoveElevatorToPosition(ElevatorGrabber elevatorGrabber, double targetPositionMeters) {
        addRequirements(elevatorGrabber);
        this.elevatorGrabber = elevatorGrabber;
        this.targetPositionMeters = targetPositionMeters;
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constants.ElevatorGrabber.elevatorMaxVelMetersPerSecond,
                Constants.ElevatorGrabber.elevatorMaxAccelMetersPerSecondSquared),
            new TrapezoidProfile.State(
                targetPositionMeters, 0.0
            ),
            new TrapezoidProfile.State(
                elevatorGrabber.getElevatorPositionMeters(), elevatorGrabber.getElevatorVelocityMetersPerSecond()
            )  
        );

        
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevatorGrabber.setElevatorMotorMetersPerSecond(
            profile.calculate(timer.get()).velocity
            + 
            );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorGrabber.setElevatorMotorMetersPerSecond(0); 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }
}
