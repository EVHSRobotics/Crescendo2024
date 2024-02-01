// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  TalonSRX top;
  TalonSRX bottom;
  static final double unitsPerRot = 4096;
  BangBangController flyWheelController = new BangBangController();

  /** Creates a new Shooter. */
  public Shooter() {
    TalonSRXConfiguration configuration = new TalonSRXConfiguration();
    // configuration.motionAcceleration = 
    top = new TalonSRX(12);
    bottom = new TalonSRX(11);
    
  //   top.config_kP(0, 0.000);
  //   top.config_kI(0, 0.000);
  //   top.config_kD(0, 0.00);
  // top.config_kF(0, 0.02046)itop\\
    top.setInverted(true);
        bottom.setInverted(true);

    bottom.follow(top);
    
  }

  public double getRPM(){
    // return 600*top.getSelectedSensorVelocity()/unitsPerRot;
    return top.getSelectedSensorVelocity();
  }

  public void setShooterSpeed(double percentOutput) {
    top.set(ControlMode.PercentOutput,percentOutput);
    bottom.set(ControlMode.PercentOutput, percentOutput);

  }

  public void setShooterRPM(double val){
    // double error = RPM - top.getSelectedSensorVelocity();
    // Multiply velocity units by 600/UnitsPerRotation to obtain RPM.

    // double Velo = RPM * unitsPerRot/600;

    double output = flyWheelController.calculate(top.getSelectedSensorVelocity(), val);
    SmartDashboard.putNumber("outputPercemtFlyWheel", output);
    top.set(TalonSRXControlMode.PercentOutput, output);
    bottom.set(TalonSRXControlMode.PercentOutput, output);

  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
