// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeIndexer.IndexerState;

public class LEDs extends SubsystemBase {
  private Spark blinkin;

  private IndexerState indexerState;
  public LEDs() {
    blinkin = new Spark(9);

    indexerState = IndexerState.cone;
  }

  public void setCubeMode() {
    indexerState = IndexerState.cube;
  }

  public void setConeMode() {
    indexerState = IndexerState.cone;
  }
  
  public boolean inConeMode() {
    return indexerState == IndexerState.cone;
  }

  public void setLEDsYellow() {
    blinkin.set(0.69);
  }

  public void setLEDsPurple() {
    blinkin.set(0.91);
  }

  public void setLEDSOrange() {
    blinkin.set(0.65);
  }

  @Override
  public void periodic() {
    

    blinkin.set(-0.63);

    SmartDashboard.putBoolean("in cone mode", true);

  }
}

