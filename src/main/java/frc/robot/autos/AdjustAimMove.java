// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Vision;
import frc.robot.commands.SuperStructure.IntakeMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AdjustAimMove extends Command {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // private boolean runningInitializeIntake = true;
  private Arm arm;
  private Shooter shoot;
  private Intake intake;
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
 private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                           

  private final double autoCalib = 0.0065;

  /** Creates a new ShootNoteAuto. */
  public AdjustAimMove(Arm arm, Shooter shoot, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.shoot = shoot;
    this.intake = intake;


    addRequirements(arm);
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drivetrain.applyRequest(() -> drive.withRotationalRate(Vision.aimLimelightObject("limelight") * MaxAngularRate)).execute(); // Drive counterclockwise with negative X (left)

    arm.setPosition(Vision.getPredTheta() + autoCalib);
    shoot.motionMagicVelo(Vision.getPredVelocity());
    intake.keepNote();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // drivetrain.applyRequest(() -> drive.withRotationalRate(Vision.aimLimelightObject("limelight") * MaxAngularRate)).execute(); // Drive counterclockwise with negative X (left)
        
    intake.keepNote();
    arm.setPosition(Vision.getPredTheta() + autoCalib);
    shoot.motionMagicVelo(Vision.getPredVelocity());

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
