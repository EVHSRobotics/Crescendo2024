// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));;

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  enum AutoPaths {
    
    StraightPath("NewPath");

    private String pathName;

    AutoPaths(String pathName) {
      this.pathName = pathName;
    }

    public String getPath() {
      return this.pathName;
    }
  }

  interface AutoCommands {
    public Command action();
  }
  

 
  public Command getAutonomousCommand() {

    // Command straightLineCommand = genAutoScript(new AutoCommands[] {
    //   new AutoCommands() {
    //     public Command action() {
    //       return genChoreoCommand(AutoPaths.StraightPath);
    //     }
    //   }
    // });

    
    // return straightLineCommand;
    return genChoreoCommand(AutoPaths.StraightPath);
  } 

  // Creates an auto script filled with commands and paths
  public Command genAutoScript(AutoCommands... comms) {
    if (comms.length == 0) { return null; }
    Command script = comms[0].action();

    for (int i = 1; i<comms.length; i++) {
      script.andThen(comms[i].action());

    }
    return script;
  }

  // public void yes(){
  //   AutoBuilder.configureHolonomic(
  //           this::getPose, // Supplier of current robot pose
  //           this::seedFieldRelative,  // Consumer for seeding pose against auto
  //           this::getCurrentRobotChassisSpeeds,
  //           (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
  //           new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
  //                                           new PIDConstants(10, 0, 0),
  //                                           1,
  //                                           1,
  //                                           new ReplanningConfig(),
  //                                           0.004),
                                            
  //           this);
  // }

  // Creates the path auto command
  public Command genChoreoCommand(AutoPaths path) {
    
    ChoreoTrajectory pathTraj = Choreo.getTrajectory(path.getPath());
    drivetrain.setPose(pathTraj.getInitialPose());
    /* First put the drivetrain into auto run mode, then run the auto */
    var thetaController = new PIDController(5, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    

      Command swerveCommand = Choreo.choreoSwerveCommand(
        pathTraj, // Choreo trajectory from above
        drivetrain::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(5, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(5, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-speeds.vxMetersPerSecond * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-speeds.vyMetersPerSecond * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-speeds.omegaRadiansPerSecond * MaxAngularRate) // Drive counterclockwise with negative X (left)
          ).ignoringDisable(true),
        () ->  DriverStation.getAlliance().get().equals(Alliance.Red), // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        drivetrain // The subsystem(s) to require, typically your drive subsystem only
    );

    

    

    return swerveCommand;
    
  }
}
