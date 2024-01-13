// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Limelight;


public class Vision extends Command {

  // private AprilScanner aprilScanner;
  private Limelight aprilLimelight;
  private CommandXboxController xboxController;
public Limelight gameObjectTopLimelight;
  public Limelight gameObjectBottomLimelight;

  double errorsum = 0;
  double lasterror = 0;
  double error;
  double lastTimestamp = 0;

  /** Creates a new Vision. */
  public Vision(Limelight aprilLimelight, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    // VideoServer videoServer, AprilScanner aprilScanner,
    // this.aprilScanner = aprilScanner;
    this.aprilLimelight = aprilLimelight;
    this.xboxController = xboxController;
    // addRequirements(videoServer);
    // addRequirements(aprilScanner);
    addRequirements(aprilLimelight);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // aprilScanner.detectAprilCode();
  


      // if (intake.gameObject == GameObject.CUBE) {

      //   aimLimelightAprilTags();
      // }
      // else if (intake.gameObject == GameObject.CONE) {
      //   aimLimelightReflective();
      // }
  
   
  }

 


  
  public void aimLimelightAprilTags() {

    double x = aprilLimelight.getX();
    errorsum = 0;
    error = x;

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (error-lasterror)/dt;
    if(Math.abs(x) < 0.1){
        errorsum += dt *  x;
    }
    double output = MathUtil.clamp(error*0.045 + errorrate *0+errorsum*0.0, -1, 1);

    SmartDashboard.putNumber("limelight", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    lasterror = error;
    
 
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
