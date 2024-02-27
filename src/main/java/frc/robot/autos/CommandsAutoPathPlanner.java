// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.sql.Time;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.SuperStructure.ArmPosition;
import frc.robot.commands.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CommandsAutoPathPlanner extends Command {
  public static enum AutoCommandsType {
    GROUND_INTAKE, SHOOT_NOTE, HANG;

  }

  
  private Arm arm;
  private Intake intake;
  private Shooter shoot;
  public AutoCommandsType commandAction;

  /** Creates a new AutoCommands. */
  public CommandsAutoPathPlanner(AutoCommandsType commandAction, Intake intake, Shooter shoot) {
    this.commandAction = commandAction;
    this.intake = intake;
    this.shoot = shoot;
  }

  // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setPosition(ArmPosition.LOW_INTAKE.getPos());
    intake.runIntake(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (commandAction) {
      case GROUND_INTAKE:
        intake.runIntake(-1);
        break;

      case SHOOT_NOTE:
        intake.runIntake(1);
        shoot.motionMagicVelo(
            NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
        arm.setPosition(
            NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(0));
        break;

      case HANG:
        break;

      default:
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (commandAction) {
      case GROUND_INTAKE:
        arm.setPosition(ArmPosition.STOW.getPos());
        shoot.motionMagicVelo(0);
        intake.runIntake(0);
        break;
      case SHOOT_NOTE:
        arm.setPosition(ArmPosition.LOW_INTAKE.getPos());
        shoot.motionMagicVelo(0);
        intake.runIntake(0);
        break;
      default:
        break;

    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
