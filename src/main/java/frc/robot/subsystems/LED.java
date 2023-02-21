// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  // 0 and 1 are the PWM ports on the RoboRIO the Blinkin and LED strips are attached to
  private Spark blinkin = new Spark(0);

  public LED() {}

  /**
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the REV Robotics Blinkin User's Manual Table 5 for a mapping of values to patterns.
   * 
   * @param value The LED blink color and patern value [-1,1]
   * 
   * @see <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14">REV Robotics Blinkin User's Manual Table 5</a>
   */
  private void setBlinkin(double value) {
    if ((value >= -1.0) && (value <= 1.0)) {
      blinkin.set(value);
    }
  }

  public void setYellow() {
    setBlinkin(0.69);
  }

  public void setPurple() {
    setBlinkin(0.91);
  }

  public void flashRed() {
    setBlinkin(-0.11);
  }

  @Override
  public void periodic() {}
}
