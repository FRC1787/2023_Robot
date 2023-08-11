// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {



    private AngledElevatorSim elevatorSim;


    public ElevatorIOSim() {


        elevatorSim = 
            new AngledElevatorSim(
                DCMotor.getNEO(1),
                1.0/Constants.ElevatorGrabber.elevatorReduction,
                3,
                Constants.ElevatorGrabber.drumRadius,
                0,
                Constants.ElevatorGrabber.elevatorMaxPositionMeters,
                true,
                60
            );

    }

    @Override
    public void extendElevator() {
        elevatorSim.setAngleDegrees(30);
    };

    @Override
    public void retractElevator() {
        elevatorSim.setAngleDegrees(60);
    };

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(0.02);

        inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
        inputs.elevatorVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();

        inputs.atLowerLimit = elevatorSim.hasHitLowerLimit();
        inputs.atUpperLimit = elevatorSim.hasHitUpperLimit();
    };


    @Override
    public void setElevatorMotorVoltage(double volts) {
        elevatorSim.setInputVoltage(volts);
    };

}
