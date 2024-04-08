// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedReader;
import java.io.Console;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.List;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SuperStructure.ArmPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Vision extends Command {

  // private AprilScanner aprilScanner;
  private XboxController operatorController;

  public double xdistance;
  NetworkTable table1;
  NetworkTableEntry tx, tv;

  static double errorsum = 0;
  static double lasterror = 0;
   static double error;
  static ProfiledPIDController visionPIDController = new ProfiledPIDController(0.3, 1, 0, new Constraints(0.15, 0.25));

  static double lastTimestamp = 0;  
  static double lastTX = 0.0;
static public double caliOffset = -0.025;

  public Vision(XboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    // VideoServer videoServer, AprilScanner aprilScanner,
    // this.aprilScanner = aprilScanner;
    // table1 = NetworkTableInstance.getDefault().getTable("limelight-bottom");
    this.operatorController = operatorController;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   
  }
  // public static double getObjectDistanceNote() {
  //   double testArea = 1.46, testDis = 68.0;
  //   double d = Math.sqrt(testArea/LimelightHelpers.getTA("limelight-intake")) * testDis;
  //   return d; 
  //   // return ((lensHeight) / Math.abs(Math.tan(Math.toRadians(getY())))/2.0);
  // }
 
  // public static  double getObjectDistanceOutputVert() {
  //   return MathUtil.clamp(((-getDistanceObject(ArmPosition.LOW_INTAKE)) * 0.05), -1, 1);
  // }


  // public static boolean doesSeeLimelightGamePiece() {
  //   double classData = LimelightHelpers.getNeuralClassID("limelight-intake");
  //   SmartDashboard.putNumber("detectedValue", classData);
  //   SmartDashboard.updateValues();
  //   if (classData == 0 || classData == 1) {
  //     return true;
  //   }
  //   else {
  //     return false;
  //   }
  // }
  
  public static void resetPIDController() {

   visionPIDController.reset(LimelightHelpers.getTX("limelight"));
    // visionPIDController.enableContinuousInput(-180, 180);


  }
  public static double getPredTheta() {
    
    // if (LimelightHelpers.getTV("limelight")) {
// return 0;
    // }
    double predictedTheta = -0.0103 + 2.38e-03*LimelightHelpers.getTY("limelight") + -3.03e-07*Math.pow(LimelightHelpers.getTY("limelight"), 2) + -3.1e-07*Math.pow(LimelightHelpers.getTY("limelight"), 3) + caliOffset;
    return predictedTheta;
  }
  public static double getPredVelocity() {
    double predictedVelocity = 69.9 + -0.999*LimelightHelpers.getTY("limelight") + 0.0529*Math.pow(LimelightHelpers.getTY("limelight"), 2);
    return predictedVelocity * 1.1;
  }
  public static double getLimelightAprilTagTXError() {
    return LimelightHelpers.getTX("limelight");
  }

  public static List<Translation2d> getRobotDetection(Translation2d currentBotPose) {
    currentBotPose.getX();
    return null;
  }
  // public static double getDistanceObject(ArmPosition pos) {
  //   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
  //   NetworkTableEntry ty = table.getEntry("ty");
  //   double targetOffsetAngle_Vertical = ty.getDouble(0.0);

  //   // how many degrees back is your limelight rotated from perfectly vertical?
  //   double limelightMountAngleDegrees = -35.0; 

  //   // distance from the center of the Limelight lens to the floor
  //   double limelightLensHeightInches = 20.0; // Low Intake

  //   if (pos == ArmPosition.HIGH_INTAKE) {
  //     limelightLensHeightInches = 60;
  //   }

  //   // distance from the target to the floor
  //   double goalHeightInches = 0.0; 

  //   double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  //   double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  //   //calculate distance
  //   double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  //   return distanceFromLimelightToGoalInches * 0.025;
  // }


  public static double aimLimelightObject(String limelightName) {



    double x = LimelightHelpers.getTX(limelightName);
    

    if(Math.abs(x) > 1){
      errorsum = 0;
      error = x;

      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      double errorrate = (error-lasterror)/dt;
 
      // double output = MathUtil.applyDeadband(MathUtil.clamp(error*0.018  + errorrate *0.002+errorsum*0.000, -1, 1), 0.02);
      // System.out.println(output);
    
          // return profiledOutput;

      lastTimestamp = Timer.getFPGATimestamp();
      lasterror = error;
      // if (limelightName.equals("limelight-intake")) {
      // return -(error*0.3+errorrate *0.003 + 0.05 * Math.signum(error));

      // }
      // else {
      return -(error*0.016+errorrate *0.003 + 0.05 * Math.signum(error));

      // }
  }
    
    // if (!tv) {
    //  if (lastTX < 0) {
    //   return 0.75;
    //  }
    //  else {
    //   return -0.75;
    //  } 
    // }
    // else {

    // }
    return 0;
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}