package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Telemetry extends SubsystemBase {

    private ShuffleboardTab tab;
    private GenericEntry intakeMotorSpeedRPM;
    private GenericEntry conveyorMotorSpeedRPM;
    private GenericEntry indexerLeftMotorSpeedRPM;
    private GenericEntry indexerRightMotorSpeedRPM;


    /*
     * Use this class to send and receive data from Shuffleboard.
     */

    public Telemetry() {
        tab = Shuffleboard.getTab("rpm calibration");
        intakeMotorSpeedRPM = tab.add("intake motor speed rpm", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        conveyorMotorSpeedRPM = tab.add("conveyor motor speed rpm", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        indexerLeftMotorSpeedRPM = tab.add("indexer left motor speed rpm", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        indexerRightMotorSpeedRPM = tab.add("indexer right motor speed rpm", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    }

    public double getIntakeMotorSpeedRPM() {
        return intakeMotorSpeedRPM.getDouble(0);
    }

    public double getConveyorMotorSpeedRPM() {
        return conveyorMotorSpeedRPM.getDouble(0);
    }

    public double getIndexerLeftMotorSpeedRPM() {
        return indexerLeftMotorSpeedRPM.getDouble(0);
    }
    
    public double getIndexerRightMotorSpeedRPM() {
        return indexerRightMotorSpeedRPM.getDouble(0);
    }

    @Override
    public void periodic() {

    }
    
}
