// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModuleIOSim implements SwerveModuleIO {
    private FlywheelSim angleSim = new FlywheelSim(DCMotor.getNEO(1), 0, 0);
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 0, 0);

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        angleSim.update(0.02);
        driveSim.update(0.02); //TODO: add 0.02 to constants as loop time

        //DRIVE MOTOR ----
        //converts rpm of drive motor into m/s of wheel
        double driveVelocityConversionFactor =  
            1./60. * Constants.Swerve.driveReduction
            * Constants.Swerve.wheelCircumferenceMeters;
        
        inputs.driveVelocityMetersPerSecond = 
            driveSim.getAngularVelocityRPM()*driveVelocityConversionFactor;

        //calculate wheel position from (wheel velocity)*dt
        inputs.drivePositionMeters +=
            inputs.driveVelocityMetersPerSecond*0.02;

        
        //ANGLE MOTOR ----

        //converts rpm of angle motor into deg/s of swerve module
        double angleVelocityConversionFactor =
            Constants.Swerve.steerReduction*360.0/60.0;

        double angleVelocityDegreesPerSecond = 
            angleSim.getAngularVelocityRPM()*angleVelocityConversionFactor;

        //calculate module angle from (angular velocity)*dt, then mod to keep within -180 and 180
        inputs.angleAbsolutePositionDegrees =
            MathUtil.inputModulus(angleVelocityDegreesPerSecond*0.02, -180, 180);

    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSim.setInputVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleSim.setInputVoltage(volts);
    }
}
