// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;


public class Climbers extends SubsystemBase {

  private TalonFX climberMotor;

  /** Creates a new Climbers. */
  public Climbers() {


    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.CurrentLimits.SupplyCurrentLimit = 40;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = 40;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;

//  configuration.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
//  configuration.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;

    configuration.HardwareLimitSwitch.ReverseLimitEnable = true;
    configuration.HardwareLimitSwitch.ForwardLimitEnable = true;
    
    climberMotor = new TalonFX(45);
    
    climberMotor.getConfigurator().apply(configuration);

  }

  public void moveClimbers(double speed) {

    
      climberMotor.set(speed);
    
  }
  
  // public void goTop(){
  //   if(!topLimitSwitch.get()){
  //     climberMotor.set(-0.75);
  //   }
  //   else {
  //     climberMotor.set(0);
  //   }
  // }

  // public void goBottom(){
  //   if(!bottomDigitalInput.get()){
  //     climberMotor.set(0.75);
  //   }
  //   else {
  //     climberMotor.set(0);
    
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

