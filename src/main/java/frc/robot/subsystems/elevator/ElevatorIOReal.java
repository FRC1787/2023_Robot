// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO {

//TODO: TEST THIS ON THE ROBOT TO MAKE SURE I DIDNT BREAK ANYTHING
    private CANSparkMax elevatorMotor;
    private RelativeEncoder encoder;
    private DigitalInput lowerLimitSwitch;
    private DoubleSolenoid solenoid;

    public ElevatorIOReal() {

        elevatorMotor = new CANSparkMax(
            Constants.ElevatorGrabber.elevatorMotorID,
            MotorType.kBrushless);
      
          solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.ElevatorGrabber.elevatorRetractPneumaticChannel,
            Constants.ElevatorGrabber.elevatorExtendPneumaticChannel
          );
      
          // encoder = elevatorMotor.getAlternateEncoder(8192);
          // encoder.setPositionConversionFactor(1);
          // encoder.setVelocityConversionFactor(1);
      
          encoder = elevatorMotor.getEncoder();
          configureMotors();

          lowerLimitSwitch = new DigitalInput(Constants.ElevatorGrabber.lowerLimitSwitchID);

        
    }

    private void configureMotors() {
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(50);
        elevatorMotor.setInverted(true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        encoder.setPositionConversionFactor(Constants.ElevatorGrabber.grabberMetersPerRotation * Constants.ElevatorGrabber.elevatorReduction);
        encoder.setVelocityConversionFactor(Constants.ElevatorGrabber.grabberMetersPerSecondPerRPM * Constants.ElevatorGrabber.elevatorReduction);
        elevatorMotor.burnFlash();
      }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.atLowerLimit = lowerLimitSwitch.get();
        inputs.atUpperLimit = (encoder.getPosition() > Constants.ElevatorGrabber.elevatorMaxPositionMeters);

        inputs.elevatorPositionMeters = encoder.getPosition();
        inputs.elevatorVelocityMetersPerSecond = encoder.getVelocity();

    };

    @Override
    public void extendElevator() {
        solenoid.set(Value.kReverse);
    };

    @Override
    public void retractElevator() {
        solenoid.set(Value.kForward);
    };

    @Override
    public void setElevatorMotorVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    };

    @Override
    public void zeroEncoder() {
        encoder.setPosition(0);
    };
    
}
