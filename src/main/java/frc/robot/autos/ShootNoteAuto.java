// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SuperStructure.IntakeMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNoteAuto extends Command {
  
  private Arm arm;
  private Intake intake;
  private Shooter shoot;
  private Timer shootTimer;

  private boolean isAutoFinished = false;

  /** Creates a new ShootNoteAuto. */
  public ShootNoteAuto(Arm arm, Intake intake, Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    this.shoot = shoot;

    shootTimer = new Timer();

    addRequirements(arm);
    addRequirements(intake);
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAutoFinished = false;

    shootTimer.purge();

    shootTimer.schedule(new TimerTask() {
      @Override
      public void run() {
        this.cancel();
        intake.pushIntake(IntakeMode.OUTTAKE.getSpeed());
        shootTimer.schedule(new TimerTask() {

          @Override
          public void run() {
            this.cancel();
            intake.pushIntake(0);
            shoot.motionMagicVelo(0);
            isAutoFinished = true;
          }

        }, 750);
      }
    }, 2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.motionMagicVelo(
            NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
    arm.setPosition(
            NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(0));
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
