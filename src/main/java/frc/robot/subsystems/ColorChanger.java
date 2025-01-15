// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lighting;
import frc.robot.Constants.Lighting.Colors;

public class ColorChanger extends SubsystemBase { 

  public AddressableLED m_led = new AddressableLED(2);//pwm port 2

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
  public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(200);

  /** Creates a new ColorChanger. */
 public ColorChanger() {
 
   

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

public void setRAINBOWRAINBOW() {
        // For every pixel
        int m_rainbowFirstPixelHue = 1;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final int hue = ((m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180);
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue  += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
}

    public void setRed(){

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
         }
         
         m_led.setData(m_ledBuffer);

    }

    public void setGreen(){

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            m_ledBuffer.setRGB(i, 0, 255, 0);
         }
         
         m_led.setData(m_ledBuffer);

    }

    public void setBlue(){

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            m_ledBuffer.setRGB(i, 111, 206, 255);
         }
         
         m_led.setData(m_ledBuffer);

    }

    public void setPurple(){

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            m_ledBuffer.setRGB(i, 200, 48, 255);
         }
         
         m_led.setData(m_ledBuffer);

    }

    public void setColor(){

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            m_ledBuffer.setRGB(i, 56, 78, 155);
         }
         
         m_led.setData(m_ledBuffer);

    }

    public void setGold(){

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            m_ledBuffer.setRGB(i, 212, 178, 25);
         }
         
         m_led.setData(m_ledBuffer);

    }
   

    public void setOrange(){

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            m_ledBuffer.setRGB(i, 235, 165, 3);
         }
         
         m_led.setData(m_ledBuffer);

    }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    // Colors selectedColor = lighting_chooser.getSelected();

    //lighting.set(selectedColor.getColorValue());
  }

  public static void set(double colorValue) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }
}
