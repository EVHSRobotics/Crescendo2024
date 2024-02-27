// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

    private Spark sparkLights;

    public enum SparkLEDColors {

      RAINBOW(-0.99), ALGO_AIM(-0.63), ALGO_SHOOT(-0.57), AMP(-0.17), LOW_INTAKE(-0.11), HIGH_INTAKE(-0.07), CLIMBING(-0.91);
    
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

    }

    public void setLED(SparkLEDColors ledColor) {
    
      sparkLights.set(ledColor.getColor());
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
