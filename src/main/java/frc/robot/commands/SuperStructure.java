// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SuperStructure extends Command {

  private Arm arm;
  private Intake intake;
  private Shooter shoot;
  private ArmPosition currentPosition = ArmPosition.STOW;
  private IntakeMode currentIntake = IntakeMode.MANUAL;

  private XboxController operator;
  private Timer intakeTimer = new Timer();

  public enum IntakeMode {

    OUTTAKE(1, 1000),
    INTAKE(-1, 1000),
    MANUAL(0, 0);

    private double speed;
    private long time; // IN MS

    IntakeMode(double speed, long time) {
      this.speed = speed;
      this.time = time;
    }

    public double getSpeed() {
      return this.speed;
    }
     public long getTime() {
      return this.time;
    }
  }

  public enum ArmPosition {

    STOW(-0.25),
    LOW_INTAKE(0.06),
    HIGH_INTAKE(-0.19),
    AMP(-0.180908),
    SHOOT(-0.02),
    STAGEFIT(0.01),
    ALGO(0),
    HORIZONTAL(0);

    private double pos;

    ArmPosition(double pos) {
      this.pos = pos;
    }

    public double getPos() {
      return this.pos;
    }
  }

  /** Creates a new SuperStructure. */
  public SuperStructure(Arm arm, Intake intake, Shooter shoot, XboxController operator) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.arm = arm;
    this.intake = intake;
    this.shoot = shoot;

    this.operator = operator;

    addRequirements(arm);
    addRequirements(intake);
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operator.getRightBumperPressed()) {
      setPosition(ArmPosition.ALGO);
    } 
    else if (operator.getRightBumperReleased()) {
      setIntakeMode(IntakeMode.OUTTAKE);
    }
    else if (operator.getAButton()) {
      setPosition(ArmPosition.LOW_INTAKE);

    }
    else if (operator.getBButton()) {
      setPosition(ArmPosition.AMP);
    } else if (operator.getYButton()) {
      setPosition(ArmPosition.STOW);
      setIntakeMode(IntakeMode.MANUAL);

    } else if (operator.getLeftBumper()) {
      setPosition(ArmPosition.HIGH_INTAKE);
      setIntakeMode(IntakeMode.INTAKE);

    }
    else if (operator.getXButton()) {
      setPosition(ArmPosition.STAGEFIT);
    }


    if(operator.getPOV() == 270){
      intake.useBanner = !intake.useBanner;
    }


    if (currentPosition == ArmPosition.ALGO && operator.getRightBumper()) {
      shoot.setShooterRPM(
          NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
      arm.setPosition(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta")
          .getDouble(ArmPosition.HIGH_INTAKE.getPos()));
    } else {
      arm.setPosition(currentPosition.getPos());
    }

    if (currentIntake == IntakeMode.MANUAL) {
      intake.runIntake(MathUtil.applyDeadband(operator.getLeftY(), 0.1));
    }
    else {
      intake.runIntake(currentIntake.getSpeed());
    }
  }

  public void setPosition(ArmPosition pos) {
    this.currentPosition = pos;
  }

  public void setIntakeMode(IntakeMode mode) {
    this.currentIntake = mode;

    if (mode != IntakeMode.MANUAL) {
      intakeTimer.schedule(new TimerTask() {
        @Override
        public void run() {
          currentIntake = IntakeMode.MANUAL;
          intakeTimer.cancel();
        }
      }, mode.getTime());
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
