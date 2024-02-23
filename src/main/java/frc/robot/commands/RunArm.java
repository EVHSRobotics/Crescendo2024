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
  private boolean currentlyAdjusting = false;
  private boolean noteIn = false;
  private final Timer m_timer = new Timer();
  public enum ArmPosition {

    
    STOW(-0.25),
    LOW_INTAKE(0.06),
    HIGH_INTAKE(-0.19),
    AMP(-0.180908),
    SHOOT(-0.02),
    STAGEFIT(0.01),
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

    if(controller.getRightBumper()){
      currentlyAdjusting = true;
      noteIn = false;
      //arm.goPosMotionMagic(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(0));
      arm.setPosition(SmartDashboard.getNumber("testingArmPos", 0));
    }
    else if(currentlyAdjusting){
      //Shoot.setShooterRPM(NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
      if (m_timer.get() < 0.6){
        Shoot.setShooterRPM(SmartDashboard.getNumber("rpmTop", 0));
      }
      else {
        m_timer.reset();
        currentlyAdjusting = false;
        arm.setPosition(ArmPosition.STOW.getPos());
      }

    }
    else if (controller.getAButton() && !noteIn) {
      
      arm.setPosition(ArmPosition.LOW_INTAKE.getPos());
      Intake.runIntake(.50);
      if(Intake.bannerseen == true){
      noteIn = false;
      arm.setPosition(ArmPosition.STOW.getPos());
      }
    }

    

    else if (controller.getBButton()) {
      arm.setPosition(ArmPosition.AMP.getPos());
      Intake.shootAmp();
    }
    else if (controller.getYButton()) {
      arm.setPosition(ArmPosition.STOW.getPos());
    }
    else if (controller.getLeftBumper() && !noteIn) {
      setPosition(ArmPosition.HIGH_INTAKE);
      Intake.runIntake(.50);
      if(Intake.bannerseen == true){
      noteIn = false;
      arm.setPosition(ArmPosition.STOW.getPos());
      }
    }

    else if (controller.getXButton()){
      arm.setPosition(ArmPosition.STAGEFIT.getPos());
    }

    else if(controller.getPOV() == 270){
      noteIn = !noteIn;
    }
    // arm.setPosition(currentPosition.getPos());
  //   if (controller.getBButton()) {
  //     arm.goPosMotionMagic(0);
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
