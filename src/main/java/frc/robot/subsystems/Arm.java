// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private TalonFX left;
  private TalonFX right;
  /** Creates a new Arm. */
  // Neg out back to base - for right and left
  // Pos out to talon fx arm - for right and left
  public Arm() {
    // -8 back to base
    // 5 to the talons
    // stragiht up is ~ 7
    // horizontal is ~ 0
    // down is ~ -1.6

    left = new TalonFX(43);
    right = new TalonFX(42);
right.setPosition(0);
    // left.setInverted(true);
    // left.setInverted(true);
    // left.set(ControlMode.Follower,)
    left.setControl(new Follower(42, true));
  }
  public double getArmPosition() {
    return right.getPosition().getValueAsDouble();
  }
  
  public void moveArm(double perOut) {
    right.setControl(new DutyCycleOut(-perOut));
  }
  public void reset(){
    right.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
