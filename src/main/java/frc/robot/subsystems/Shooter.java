// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  TalonFX top;
  TalonFX bottom;
  static final double unitsPerRot = 4096;
  private final double MAXSPEED = 500;

  /** Creates a new Shooter. */
  public Shooter() {
    top = new TalonFX(40);
    bottom = new TalonFX(41);

    top.setNeutralMode(NeutralModeValue.Coast);
    bottom.setNeutralMode(NeutralModeValue.Coast);

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    var slot0Configs = configuration.Slot0;

    slot0Configs.kG = 0;
    slot0Configs.kS = 0.27002; // static friction
    slot0Configs.kV = 0.11435; // velocity
    slot0Configs.kA = 0; // aceleartion
    slot0Configs.kP = 0.33;
    slot0Configs.kI = 0; // 0.05
    slot0Configs.kD = 0; // 0.1

    var motionMagic = configuration.MotionMagic;

    motionMagic.MotionMagicAcceleration = 200;
    motionMagic.MotionMagicJerk = 2000;
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configuration.CurrentLimits.SupplyCurrentLimit = 90;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = 90;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;

    top.getConfigurator().apply(configuration);
    bottom.getConfigurator().apply(configuration);

  }

  public Double getVelocity() {
    return top.getVelocity().getValueAsDouble();
  }

  public Double topGetVelocity() {
    return top.getVelocity().getValueAsDouble();
  }

  public void topVoltageConsumer(Double voltage) {
    SmartDashboard.putNumber("shooterVoltageFFChar", voltage);
    SmartDashboard.updateValues();
    top.setControl(new VoltageOut(voltage));
  }

  // double error = RPM - top.getSelectedSensorVelocity();
  // Multiply velocity units by 600/UnitsPerRotation to obtain RPM.
  // unitsPerRot = 4096, Maxspeed =500
  // double Velo = RPM * unitsPerRot/600;

  public void motionMagicVelo(double velocityOutput) {
    top.setControl(new MotionMagicVelocityVoltage(velocityOutput));
    bottom.setControl(new MotionMagicVelocityVoltage(velocityOutput));

  }

  public void motionMagicVelo(double topSpeed, double bottomSpeed) {
    top.setControl(new MotionMagicVelocityVoltage(topSpeed));
    bottom.setControl(new MotionMagicVelocityVoltage(bottomSpeed));
    System.out.println(top.getVelocity().getValueAsDouble() - bottom.getVelocity().getValueAsDouble());

  }

  public void stopShooters() {
    top.setControl(new MotionMagicVelocityVoltage(0));
    bottom.setControl(new MotionMagicVelocityVoltage(0));
  }

  public void setPower(double power) {
    top.setControl(new DutyCycleOut(power));
  }

  public void setRPS(double RPS) {
    top.setControl(new MotionMagicVelocityVoltage(RPS));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
