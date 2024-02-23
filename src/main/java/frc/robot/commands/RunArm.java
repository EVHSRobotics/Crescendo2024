// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunArm extends Command {

  private Arm arm;
  private XboxController controller;
  private Intake Intake;
  private Shooter Shoot;
  private ArmPosition currentPosition = ArmPosition.STOW;


  private final Timer m_timer = new Timer();
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
  /** Creates a new RunArm. */
  public RunArm(Arm arm, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.Shoot = new Shooter();
    this.Intake = new Intake();
    this.controller = controller;
    addRequirements(arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("armpos", arm.getArmPosition());
    // SmartDashboard.putNumber("edit", 0);
    // SmartDashboard.putNumber("numberMG", 0);
    SmartDashboard.updateValues();

    if(controller.getRightBumperPressed()){
      // currentlyAdjusting = true;

      // //arm.goPosMotionMagic(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(0));
      // arm.setPosition(SmartDashboard.getNumber("testingArmPos", 0));
      // Intake.bannerseen = false;
      
      setPosition(ArmPosition.ALGO);
    }
    else if(controller.getRightBumperReleased()){
      //Shoot.setShooterRPM(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
      //arm.setPosition(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(0));
      // Shoot.setShooterRPM(SmartDashboard.getNumber("testingshooterspeed", 0));
      // Shoot.setShooterRPM(SmartDashboard.getNumber("rpmTop", 0));
      Intake.runIntake(1.0);
      }

    
    else if (controller.getAButton()) {
      
      setPosition(ArmPosition.LOW_INTAKE);
      Intake.runIntake(controller.getLeftY());
      // if(Intake.bannerseen == true){
      // setPosition(ArmPosition.STOW);
      // }
    }

    else if (controller.getBButton()) {
      setPosition(ArmPosition.AMP);
    }
    else if (controller.getYButton()) {
      setPosition(ArmPosition.STOW);
    }
    else if (controller.getLeftBumper()) {
      
      setPosition(ArmPosition.HIGH_INTAKE);
      Intake.runIntake(.50);
     
    }

    else if(controller.getLeftBumperReleased()){
      Intake.runIntake(0);
    }

    else if (controller.getXButton()){
      setPosition(ArmPosition.STAGEFIT);
    }

    

    if (currentPosition == ArmPosition.ALGO && controller.getRightBumper()) {
      Shoot.setShooterRPM(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
      arm.setPosition(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(ArmPosition.HIGH_INTAKE.getPos()));
    }
    else{
    arm.setPosition(currentPosition.getPos());
    }
  //   if (controller.getBButton()) {
  //     arm.goPosMotonMagic(0);
  //     System.out.println("runnign");
  //   // (ArmPosition.STOW.getPos());
  //   }
  //   else if (controller.getAButton()) {
  //   arm.goPosMotionMagic(-0.25);
  //   // (ArmPosition.STOW.getPos());
  //   } 
  //   else if (controller.getXButton()) {
  //     arm.goPosMotionMagic(-0.178);
  //   } 
  //   else if (controller.getYButton()) {
  //     arm.goPosMotionMagic(SmartDashboard.getNumber("numberMG", 0));
  //   }
  //   else {
  //   // arm.setVoltage(controller.getRightY());
  //   arm.setVoltage(0);
  //  }
    

    }
  

  public void setPosition(ArmPosition pos) {
    this.currentPosition = pos;
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
