// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
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

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightHelpers;

public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  private Command[] teleCommands;

  private boolean isUsingLimelight = false;
  private RobotContainer m_robotContainer;
  
 StructPublisher<Pose2d> publisher;
  Pose2d poseA;
  StructArrayPublisher<Pose2d> arrayPublisher;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  // WPILib
  publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

  arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    publisher.set(poseA);
    arrayPublisher.set(new Pose2d[] {poseA});

    // SmartDashboard.putBoolean("color", DriverStation.getAlliance().get().equals(Alliance.Blue));
    // SmartDashboard.updateValues();
    
    if (isUsingLimelight) {    
      var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
      
      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      if (lastResult.valid) {
        TunerConstants.DriveTrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      }
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
  public void teleopPeriodic() {}

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
