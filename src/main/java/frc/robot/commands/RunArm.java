// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RunArm extends Command {

  private Arm arm;
  private XboxController controller;
  /** Creates a new RunArm. */
  public RunArm(Arm arm, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.controller = controller;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("armpos", arm.getArmPosition());
    SmartDashboard.updateValues();
  
    // if (controller.getBButton()) {
    //   arm.setPosition(0);

    // }
    // else if (controller.getAButton()) {
    //   arm.setPosition(0.15);
    // }
    // else {
      
    //   if (arm.getArmPosition() >= -0.068) {
    //     arm.setVoltage(0.25);
    //   }
    //   else {
    //     arm.setVoltage(0);
    //   }
    // }
    //   // arm.moveArm(0);
    // }
    // if(controller.getBButton())
    // //   arm.setVoltage(SmartDashboard.getNumber("KG", 0));
    // // else if (controller.getAButton()) 
    //   arm.setVoltage(1.6);
    // else
    //   arm.setVoltage(0);
    // arm.moveArm(MathUtil.applyDeadband(controller.getLeftY(), 0.1));
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
