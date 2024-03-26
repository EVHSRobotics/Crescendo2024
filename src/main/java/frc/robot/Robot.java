// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightHelpers;

public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  private Command[] teleCommands;

  private boolean isUsingLimelight = true;
  private RobotContainer m_robotContainer;

  private boolean hasRun = false;
  
 StructPublisher<Pose2d> publisher;
  StructPublisher<Pose2d> limelightPublisher;

  Pose2d poseA;
  StructArrayPublisher<Pose2d> arrayPublisher;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  // WPILib
  limelightPublisher = NetworkTableInstance.getDefault().getTable("pose")
    .getStructTopic("LimelightPose", Pose2d.struct).publish();

publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

    arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();


    Pathfinding.setPathfinder(new LocalADStar());
    // NetworkTableInstance.getDefault().getEntry("priorityid").getDoubleArray(new Double[4,7]);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    publisher.set(poseA);

    arrayPublisher.set(new Pose2d[] {poseA});

    // SmartDashboard.putBoolean("color", DriverStation.getAlliance().get().equals(Alliance.Blue));
    // SmartDashboard.updateValues();
    
    if (isUsingLimelight) {   
      updatePoseEstimatorWithVisionBotPose(); 
      // var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
      
      // Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      // if (lastResult.valid) {
      //   limelightPublisher.set(llPose);
        
        // TunerConstants.DriveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        // TunerConstants.DriveTrain.setPose(llPose);
        // TunerConstants.DriveTrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      // }
    }
    
  }
  public void updatePoseEstimatorWithVisionBotPose() {
    try {
          var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
           
          double latency = lastResult.latency_capture;
          Pose2d visionBotPose = lastResult.getBotPose2d_wpiBlue();

    limelightPublisher.set(visionBotPose);
          // invalid LL data
    if (visionBotPose.getX() == 0.0) {
      return;
    }

    // distance from current pose to vision estimated pose
    double poseDifference = TunerConstants.DriveTrain.getPose().getTranslation()
        .getDistance(visionBotPose.getTranslation());

    if (LimelightHelpers.getTV("limelight")) {
      double xyStds;
      double degStds;
      // multiple targets detected
      if (lastResult.targets_Fiducials.length >= 2) {
        xyStds = 0.5;
        degStds = 6;
      }
      // 1 target with large area and close to estimated pose
      else if (LimelightHelpers.getTA("limelight") > 0.8 && poseDifference < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      }
      // 1 target farther away and estimated pose is close
      else if (LimelightHelpers.getTA("limelight") > 0.1 && poseDifference < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      }
      // conditions don't match to add a vision measurement
      else {
        return;
      }

      TunerConstants.DriveTrain.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      TunerConstants.DriveTrain.addVisionMeasurement(visionBotPose,
          Timer.getFPGATimestamp() - latency);
    }
  }
  catch (Exception e) {
    System.out.println("ERROR");
  }
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    if(!hasRun)
      new PathPlannerAuto("InitAuto").ignoringDisable(true).schedule();
      hasRun = true;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

     if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_autonomousCommand = new WaitCommand(0.01).andThen(m_robotContainer.getAutonomousCommand());

    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    teleCommands = m_robotContainer.getTeleCommand();
    for(Command command : teleCommands){
      command.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
      // if (isUsingLimelight) {    
      // var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
      
      // Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      // if (lastResult.valid) {

      //   // TunerConstants.DriveTrain.setPose(llPose);
      //   // TunerConstants.DriveTrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      // }
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    // m_robotContainer.getFFArm().schedule();
    // m_robotContainer.getFFSteer().schedule();
    // m_robotContainer.getSystemsCheck().schedule();
  }


  @Override
  public void testPeriodic() {


  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
