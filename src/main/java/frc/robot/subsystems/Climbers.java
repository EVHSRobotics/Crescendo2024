// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbers extends SubsystemBase {

  private CANSparkMax spkL;
  private CANSparkMax spkR;

  /** Creates a new Climbers. */
  public Climbers() {

    spkL = new CANSparkMax(0, MotorType.kBrushless);
    spkR = new CANSparkMax(1, MotorType.kBrushless);
    spkL.follow(spkR, true);

  }

  public void moveClimbers(double speed) {
    spkR.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

