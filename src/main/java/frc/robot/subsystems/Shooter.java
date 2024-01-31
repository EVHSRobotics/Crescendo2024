// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  TalonSRX top;
  TalonSRX bottom;
  

  /** Creates a new Shooter. */
  public Shooter() {
    TalonSRXConfiguration configuration = new TalonSRXConfiguration();
    // configuration.motionAcceleration = 
    top = new TalonSRX(12);
    bottom = new TalonSRX(11);
    bottom.follow(top);
   
  }

  public double getRPM(){
    return top.getSelectedSensorVelocity();
  }

  public void setShooterSpeed(double percentOutput) {
    top.set(ControlMode.PercentOutput,percentOutput);
  }

  public void setShooterRPM(double RPM){
    double error = RPM - top.getSelectedSensorVelocity();

    top.set(TalonSRXControlMode.PercentOutput, error * 0.0001);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
