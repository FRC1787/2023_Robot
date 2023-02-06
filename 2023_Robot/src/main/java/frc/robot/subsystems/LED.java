// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  // 0 and 1 are the PWM ports on the RoboRIO the Blinkin and LED strips are attached to
  private Spark blinkin = new Spark(0);
  private AddressableLED pwmLED = new AddressableLED(1);
  // buffer is equal to LEDs
  private AddressableLEDBuffer pwmLEDBuffer = new AddressableLEDBuffer(60);
  public LED() {
    pwmLED.setLength(pwmLEDBuffer.getLength());
    pwmLED.setData(pwmLEDBuffer);
    pwmLED.start();
  }

  /**
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the REV Robotics Blinkin User's Manual Table 5 for a mapping of values to patterns.
   * 
   * @param value The LED blink color and patern value [-1,1]
   * 
   * @see <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14">REV Robotics Blinkin User's Manual Table 5</a>
   */
  public void setBlinkin(double value) {
    if ((value >= -1.0) && (value <= 1.0)) {
      blinkin.set(value);
    }
  }

  /**
   * Sets the color value for all pwm LEDs using RGB
   * @param red the r value [0, 255]
   * @param green the g value [0, 255]
   * @param blue the b value [0, 255]
   */
  public void pwmGlowRGB(int red, int green, int blue) {
    for (var i = 0; i < pwmLEDBuffer.getLength(); i++) {
      pwmLEDBuffer.setRGB(i, red, blue, green);
    }
    pwmLED.setData(pwmLEDBuffer);
  }

  /**
   * Sets the color value for all pwm LEDs using HSV
   * @param hue the h value [0, 180)
   * @param saturation the s value [0, 255]
   * @param value the v value [0, 255]
   */
  public void setpwmHSV(int hue, int saturation, int value) {
    for (var i = 0; i < pwmLEDBuffer.getLength(); i++) {
      pwmLEDBuffer.setHSV(i, hue, saturation, value);
    }
    pwmLED.setData(pwmLEDBuffer);
  }

  @Override
  public void periodic() {}
}
