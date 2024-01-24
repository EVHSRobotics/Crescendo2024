// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  
  private Talon middle;
   private DigitalInput banner;
  /** Creates a new IntakeSub. */
  public IntakeSub() {

    middle = new Talon(0);

    this.banner = new DigitalInput(0);

  }
  public boolean getBanner() {
    return banner.get();
  }
  public void setIntakeSpeed(double percentOutput) {
    middle.set(percentOutput);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
