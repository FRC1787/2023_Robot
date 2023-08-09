// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public boolean atLowerLimit = false;
        public boolean atUpperLimit = false;

        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityMetersPerSecond = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setElevatorMotorVoltage(double volts) {};
    
}
