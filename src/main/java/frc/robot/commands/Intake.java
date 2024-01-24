// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class Intake extends Command {

  private IntakeSub intakeSub;

 

  private XboxController operatorController;

  /** Creates a new Intake. */
  public Intake(IntakeSub intakeSub, XboxController operator) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intakeSub = intakeSub;

    this.operatorController = operator;
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intakeSub.getBanner()) {
    intakeSub.setIntakeSpeed(operatorController.getRightTriggerAxis() > 0 ? operatorController.getRightTriggerAxis() : -operatorController.getLeftTriggerAxis());
  }
  else {
    intakeSub.setIntakeSpeed(-operatorController.getLeftTriggerAxis());
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
