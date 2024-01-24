// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  Talon forwardTop;
  Talon forwardBottom;

  /** Creates a new Shooter. */
  public Shooter() {

    forwardTop = new Talon(0);
    forwardBottom = new Talon(0);

    forwardTop.addFollower(forwardBottom);

    forwardBottom.setInverted(true);
  }

  public void setShooterSpeed(double percentOutput) {
    forwardTop.set(percentOutput);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
