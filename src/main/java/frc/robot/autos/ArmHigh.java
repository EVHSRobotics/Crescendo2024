// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SuperStructure.ArmPosition;
import frc.robot.commands.SuperStructure.IntakeMode;
import frc.robot.subsystems.Arm;

public class ArmHigh extends Command {

    private Arm arm;
    private boolean isAutoFinished = false;
  private Timer armTimer;

  /** Creates a new GroundArm. */
  public ArmHigh(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.arm = arm;
    this.armTimer = new Timer();

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   arm.setPosition(ArmPosition.HIGH_INTAKE.getPos());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

       arm.setPosition(ArmPosition.HIGH_INTAKE.getPos());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAutoFinished;
  }
}
