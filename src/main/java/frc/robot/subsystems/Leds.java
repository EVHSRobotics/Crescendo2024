// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

    private Spark sparkLights;
    // private Spark sparkLightsBottom;

    private CANdle leds;

    public enum SparkLEDColors {

      RAINBOW(-0.99), ALGO_AIM(-0.63), ALGO_SHOOT(-0.57), AMP(-0.23), LOW_INTAKE(-0.05), HIGH_INTAKE(-0.07), CLIMBING(-0.91), LOW_INTAKE_TAKEN(-0.91);
    
      private double ledColorValue;
    
      public double getColor() {
        return ledColorValue;
      }
    
      private SparkLEDColors(double ledColor) {
        this.ledColorValue = ledColor;
      }
    
    }

    /** Creates a new Leds. */
    public Leds() {
      sparkLights = new Spark(0);
      // sparkLightsBottom = new Spark(1);
      leds = new CANdle(55);
      leds.configLEDType(LEDStripType.RGB);
    

    }

    public void setLED(SparkLEDColors ledColor) {
    
      leds.clearAnimation(0);

      sparkLights.set(ledColor.getColor());
      // sparkLightsBottom.set(ledColor.getColor());
    switch (ledColor) {
      case ALGO_AIM:
        leds.setLEDs(255, 0, 0);
        break;
      
      case ALGO_SHOOT:
        leds.setLEDs(0, 255, 0);
        break;
      case AMP:
        leds.setLEDs(0, 255, 0);
        break;
      case CLIMBING:
        leds.setLEDs(0, 0, 255);
        break;
      case HIGH_INTAKE:
        leds.setLEDs(255, 255, 0);
        break; 
      case LOW_INTAKE:
        leds.setLEDs(160, 32, 240);
        break;
      case LOW_INTAKE_TAKEN:
        leds.setLEDs(0, 0, 255);
        break;
      case RAINBOW:
        leds.setLEDs(0, 0, 0);
        break;
      default:
        leds.setLEDs(255, 0, 0);
        break;
    }
  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
