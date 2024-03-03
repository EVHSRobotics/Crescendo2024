// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedReader;
import java.io.Console;
import java.io.IOException;
import java.io.InputStreamReader;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  static double lastTimestamp = 0;


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


  public static double getAimRotation() {
    double distance = LimelightHelpers.getTX("limelight");
    double error = 0.1; 
    double p_constant = -0.028; 
    double output = (distance - error) * p_constant;
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.updateValues();
    return output;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   
  }
  public static double getObjectDistanceNote() {
    double testArea = 1.46, testDis = 68.0;
    double d = Math.sqrt(testArea/LimelightHelpers.getTA("limelight-intake")) * testDis;
    return d; 
    // return ((lensHeight) / Math.abs(Math.tan(Math.toRadians(getY())))/2.0);
  }
 
  public static  double getObjectDistanceOutputVert() {
    return MathUtil.clamp(((-getObjectDistanceNote()) * 0.02), -1, 1);
  }


  public static boolean doesSeeLimelightGamePiece() {
    double classData = LimelightHelpers.getNeuralClassID("limelight-intake");
    SmartDashboard.putNumber("detectedValue", classData);
    SmartDashboard.updateValues();
    if (classData == 0 || classData == 1) {
      return true;
    }
    else {
      return false;
    }
  }
  
  public static double getLimelightAprilTagTXError() {
    return LimelightHelpers.getTX("limelight");
  }
  public static double aimLimelightObject(String limelightName) {

    double x = LimelightHelpers.getTX(limelightName);
  
    errorsum = 0;
    error = x;

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (error-lasterror)/dt;
    if(Math.abs(x) < 0.1){
        errorsum += dt *  x;
    }
    double output = MathUtil.applyDeadband(MathUtil.clamp(error*0.0165 + errorrate *0.005+errorsum*0, -1, 1), 0.05);

    SmartDashboard.putNumber("limelight", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    lasterror = error;
    
    return -output;
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