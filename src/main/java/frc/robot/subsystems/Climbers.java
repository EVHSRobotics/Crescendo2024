// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.NeutralModeValue;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Climbers extends SubsystemBase {

  private TalonSRX climberMotor;

  /** Creates a new Climbers. */
  public Climbers() {

    climberMotor = new TalonSRX(45);
    climberMotor.configVoltageCompSaturation(11);
    climberMotor.enableVoltageCompensation(true);

  }

  public void moveClimbers(double speed) {
    climberMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

