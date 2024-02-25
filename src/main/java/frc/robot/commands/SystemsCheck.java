// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SuperStructure.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SystemsCheck extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private FieldCentric drive;
  private Arm arm;
  private Intake intake;
  private Shooter shoot;

  private CHECKS[] checks = new CHECKS[] {CHECKS.ARM_STOW, CHECKS.ARM_SOURCE, CHECKS.ARM_LOW_INTAKE, CHECKS.ARM_AMP, CHECKS.PI_PING, CHECKS.INTAKE, CHECKS.OUTTAKE, CHECKS.SHOOTER};
  private int currentCheck = 0;
  private double checkTimer = -1;
  private boolean checkDone = false;

  public static enum CHECKS {
      
    DRIVE("DRIVE"), ARM_STOW("ARM_STOW"), ARM_SOURCE("ARM_SOURCE"), ARM_LOW_INTAKE("ARM_LOW_INTAKE"), ARM_AMP("ARM_AMP"), PI_PING("RASPBERRYPI_PING"), INTAKE("INTAKE"), OUTTAKE("OUTTAKE"), SHOOTER("SHOOTER");

    private String checkName;

    CHECKS(String checkName) {
      this.checkName = checkName;
    }

    public String getCheckName() {
      return this.checkName;
    }
  }

  /** Creates a new SystemsCheck. */
  public SystemsCheck(CommandSwerveDrivetrain drivetrain, FieldCentric drive, Arm arm, Intake intake, Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.arm = arm;
    this.intake = intake;
    this.shoot = shoot;

    addRequirements(arm);
    addRequirements(intake);
    addRequirements(shoot);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    checkTimer = -1;
    currentCheck = 0;
    checkDone = false;

    System.out.println("Intializing System Checks");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (!checkDone) {
      // Checks if the checks are done
      if (currentCheck >= checks.length) {
        checkDone = true;
        System.out.println("ALL SYSTEMS CHECK PASSED");
        return;
      }
      
     try {
  
      switch (checks[currentCheck]) {
        case DRIVE:
          // Run Modules at 40% speed for 3 seconds
            if ((System.currentTimeMillis() - checkTimer) < 3000) {
              drivetrain.applyRequest(() -> drive.withVelocityX(2.4).withVelocityY(2.4).withRotationalRate(1.884));
          }
          else {
            nextSystemCheck();
          }
          break;
        case ARM_AMP:
          if (!arm.isArmInRange(ArmPosition.AMP)) {
            arm.setPosition(ArmPosition.AMP.getPos());
          }
          else { 
            nextSystemCheck();
          }
        case ARM_LOW_INTAKE:
          if (!arm.isArmInRange(ArmPosition.LOW_INTAKE)) {
            arm.setPosition(ArmPosition.LOW_INTAKE.getPos());
          }
          else { 
            nextSystemCheck();
          }
        case ARM_SOURCE:
          if (!arm.isArmInRange(ArmPosition.HIGH_INTAKE)) {
            arm.setPosition(ArmPosition.HIGH_INTAKE.getPos());
          }
          else { 
            nextSystemCheck();
          }
        case ARM_STOW:
          if (!arm.isArmInRange(ArmPosition.STOW)) {
            arm.setPosition(ArmPosition.STOW.getPos());
          }
          else { 
            nextSystemCheck();
          }
        case INTAKE:
          if (intake.getIntakeSpeed() <= 0) {
            intake.pushIntake(1);
          }
          else { 
            nextSystemCheck();
          }
        case OUTTAKE:
          if (intake.getIntakeSpeed() >= 0) {
            intake.pushIntake(-1);
          }
          else { 
            nextSystemCheck();
          }
        case PI_PING:
          // If it has updated once in the last 5 seconds that means the PI is running
          if (System.currentTimeMillis() - NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTimestamp").getDouble(0) <= 5000) {
            nextSystemCheck();
          }
        case SHOOTER:
          
          if (shoot.getVelocity() >= 70) {
            shoot.motionMagicVelo(80);
          }
          else { 
            nextSystemCheck();
          }
        default:
          break;
            
      }

      if (checkTimer == -1) {
        checkTimer = System.currentTimeMillis();
        System.out.println("Currently Testing " + checks[currentCheck].getCheckName() + " (" + (currentCheck+1) + "/" + checks.length + ")");
      }
      else if ((System.currentTimeMillis() - checkTimer) > 5000) {
        checkDone = true;
        throw new Exception(checks[currentCheck].getCheckName() + " FAILED");
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
   }
  }

  public void nextSystemCheck() {
    // First Reset Systems
    intake.pushIntake(0);
    shoot.motionMagicVelo(0);
    arm.setPosition(ArmPosition.HORIZONTAL.getPos());
    drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    System.out.println(checks[currentCheck].getCheckName() + " PASSED");

    checkTimer = -1;
    currentCheck +=1;
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
