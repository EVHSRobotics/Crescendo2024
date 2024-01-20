// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private NetworkTable LimelightTable;
  public NetworkTableEntry tx;
  private NetworkTableEntry ty;
  public NetworkTableEntry tv;
  private NetworkTableEntry ta;
  private NetworkTableEntry tclass;
  private int pipeline;

  private double mountAngle; // Degrees
  private double lensHeight = 10; // Inches
  // Target Height

  /** Creates a new Limelight. */
  public Limelight(int limelightNumber) {
    // we can use the limelight number to create each specific limelight based on the location (currently just have one )
    LimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    

  }

  @Override
  public void periodic() {

    tx = LimelightTable.getEntry("tx");
    ty = LimelightTable.getEntry("ty"); 
    tv = LimelightTable.getEntry("tv");
    ta = LimelightTable.getEntry("ta");



    // Only for vision based limelight
    tclass = LimelightTable.getEntry("tclass");

    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));

    SmartDashboard.updateValues();

  }
  
  // gets the error of the limelight to the detected object
  public double getArea() {
    double a = ta.getDouble(0.0);
   
    if (tv.getBoolean(false)) {
      a = 0;
    }
    return a;   
  }

  public double getX() {
    double x = tx.getDouble(0.0);
   
    if (tv.getBoolean(false)) {
      x = 0;
    }
    return x;   
  }

  public boolean getObjectSeen() {
    return tv.getInteger(0) == 0 ? false : true;
  }

  // gets the error of the limelight to the detected object

  public double getY() {
    double y = ty.getDouble(0.0);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.updateValues();
   
    if (tv.getBoolean(false)) {
      y = 0;
    }
    return y;   
  }

  public double getObjectDistanceCone() {
    double testArea = 17.7, testDis = 28.0;
    double d = Math.sqrt(testArea/getArea()) * testDis;
    return d; 
    // return ((lensHeight) / Math.abs(Math.tan(Math.toRadians(getY())))/2.0);
  }
  public double getObjectDistanceCube() {
    double testArea = 14.8, testDis = 26.0;
    double d = Math.sqrt(testArea/getArea()) * testDis;
    return d; 
    // return ((lensHeight) / Math.abs(Math.tan(Math.toRadians(getY())))/2.0);
  }
}
