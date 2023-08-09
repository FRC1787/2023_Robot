// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

    Mechanism2d mech;
    MechanismRoot2d root;
    MechanismLigament2d elevator;

    private ElevatorSim elevatorSim;


    public ElevatorIOSim() {
        mech = new Mechanism2d(1, 1);
        root = mech.getRoot("hi", 0.35, 0.0);
        elevator = root.append(new MechanismLigament2d("elevator", 0.1, 30));

        elevatorSim = 
            new ElevatorSim(
                DCMotor.getNEO(1),
                1.0/Constants.ElevatorGrabber.elevatorReduction,
                3,
                Constants.ElevatorGrabber.drumRadius,
                0,
                Constants.ElevatorGrabber.elevatorMaxPositionMeters,
                false
            );

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(0.02);

        inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
        inputs.elevatorVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();

        inputs.atLowerLimit = elevatorSim.getPositionMeters() <= 0;
        inputs.atUpperLimit = elevatorSim.getPositionMeters() >= Constants.ElevatorGrabber.elevatorMaxPositionMeters;
    };


    @Override
    public void setElevatorMotorVoltage(double volts) {
        elevatorSim.setInputVoltage(volts);
    };

}
