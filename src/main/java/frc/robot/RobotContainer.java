// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.autos.GroundArm;
import frc.robot.autos.ShootNoteAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.autos.GroundIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.SuperStructure;
import frc.robot.commands.SystemsCheck;
import frc.robot.commands.Vision;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final XboxController operator = new XboxController(1);
  private final XboxController driver = new XboxController(0);
  private final SendableChooser<String> autoChooser;
  private Timer m_timer = new Timer();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // private Vision vision;
  // private RunArm runArm;
  // private RunIntake intake;
  private Shoot shoot;
  private SuperStructure superStructure;
  private Shooter shootSub;
  private Intake intakeSub;
  private Arm arm;
  private Leds ledSub;

  private FeedForwardCharacterization armCharacterization;
  private FeedForwardCharacterization shootTopCharacterization;
  // private FeedForwardCharacterization shootBottomCharacterization;
  private FeedForwardCharacterization driveCharacterization;
  private FeedForwardCharacterization steerCharacterization;
  

  private SystemsCheck systemsCheck;

  // private FieldCentricFacingAngle ampAngle = new FieldCentricFacingAngle()
  //     .withTargetDirection(Rotation2d.fromDegrees(270)).withVelocityX(0.75 * MaxSpeed).withVelocityY(0.75 * MaxSpeed);
  // private FieldCentricFacingAngle sourceAngle = new FieldCentricFacingAngle()
  //     .withTargetDirection(Rotation2d.fromDegrees(38)).withVelocityX(0.75 * MaxSpeed).withVelocityY(0.75 * MaxSpeed);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
        // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
       
  //  joystick.pov(90).whileTrue(drivetrain.moveToHeading(58, , null));

    // joystick.x().whileTrue(drivetrain.applyRequest(() -> sourceAngle));
    // reset the field-centric heading on right bumper press
    joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    autoChooser = new SendableChooser<String>();
    autoChooser.addOption(AutoPaths.BackupHPAuto.pathName, AutoPaths.BackupHPAuto.pathName);
    autoChooser.addOption(AutoPaths.BackupPathPlannerHPAuto.pathName, AutoPaths.BackupPathPlannerHPAuto.pathName);

    driveCharacterization = new FeedForwardCharacterization(drivetrain, drivetrain::driveWithVoltage, drivetrain::getVeloDrive);
    steerCharacterization = new FeedForwardCharacterization(drivetrain, drivetrain::steerWithVoltage, drivetrain::getVeloSteer);


    intakeSub = new Intake();
    arm = new Arm();
    shootSub = new Shooter();
    ledSub = new Leds();
    shoot = new Shoot(shootSub, operator);

    superStructure = new SuperStructure(arm, intakeSub, shootSub, ledSub, driver, operator);
    // runArm = new RunArm(arm, operator);
    // intake = new RunIntake(intakeSub, operator, runArm);
    // shoot = new Shoot(shootSub, operator, runArm);

    SmartDashboard.putData(autoChooser);
    SmartDashboard.updateValues();

    configureBindings();
    setUpAutoCommands();

    armCharacterization = new FeedForwardCharacterization(arm, arm::setVoltage, arm::getVelocity);
    // shootBottomCharacterization = new FeedForwardCharacterization(shootSub,
    // shootSub::bottomVoltageConsumer, shootSub::bottomGetVelocity);
    shootTopCharacterization = new FeedForwardCharacterization(shootSub, shootSub::topVoltageConsumer,
        shootSub::topGetVelocity);
    systemsCheck = new SystemsCheck(drivetrain, drive, arm, intakeSub, shootSub);
    // driveCharacterization = new FeedForwardCharacterization(null, null, null);
  }

  public enum AutoPaths {

    BackupHPAuto("Backup_Middle"),
    BackupPathPlannerHPAuto("BackUp");

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

  public FeedForwardCharacterization getFFShooterTop() {
    return shootTopCharacterization;
  }

  // public FeedForwardCharacterization getFFShooterBottom(){
  // return shootBottomCharacterization;
  // }
  public FeedForwardCharacterization getFFArm() {
    return armCharacterization;
  }

 public FeedForwardCharacterization getFFDrive() {
    return driveCharacterization;
  }

  public FeedForwardCharacterization getFFSteer() {
    return steerCharacterization;
  }

  public Command[] getTeleCommand() {
    Command[] commands = { superStructure };
    return commands;
  }
  public void setUpAutoCommands() {
    HashMap<String, Command> eventMap = new HashMap<String, Command>();
    eventMap.put("Arm_Ground", new GroundArm(arm));
    eventMap.put("Intake", new GroundIntake(intakeSub));
    eventMap.put("Outtake", new ShootNoteAuto(arm, intakeSub, shootSub));

    NamedCommands.registerCommands(eventMap);
  }

  public Command getAutonomousCommand() {

    // Command straightLineCommand = genAutoScript(new AutoCommands[] {
    // new AutoCommands() {
    // public Command action() {
    // return genChoreoCommand(AutoPaths.StraightPath);
    // }
    // }
    // });

    // return straightLineCommand;
    // return genChoreoCommand(AutoPaths.StraightPath);

    // easiest way: running auto through PathPlannerLib while using choreo
    // trajectories
    
    String auto = "BackUp_Middle_Left";
    drivetrain.setPose(PathPlannerAuto.getStaringPoseFromAutoFile(auto));
    return drivetrain.getAutoPath(auto);
    // drivetrain.setPose(null);
    // Command chooser = autoChooser.getSelected().;
    // return autoChooser.getSelected();
  }

  // Choreo path command using pathplanner: not needed when using auto straight
  // off of pathplanner
  public Command choreoPathPlannerGen(AutoPaths path) {
    PathPlannerPath PPpath = PathPlannerPath.fromChoreoTrajectory(path.getPath());
    drivetrain.setPose(PPpath.getStartingDifferentialPose());
    return AutoBuilder.followPath(PPpath);
  }

  public Command getSystemsCheck() {
    return systemsCheck;
  }

  // Deprecated
  // Creates an auto script filled with commands and paths
  public Command genAutoScript(AutoCommands... comms) {
    if (comms.length == 0) {
      return null;
    }
    Command script = comms[0].action();

    for (int i = 1; i < comms.length; i++) {
      script.andThen(comms[i].action());

    }
    return script;
  }

  // Creates the path auto command
  // note: was not working so moved on to using pathplanner one instead but with
  // choreo paths
  // public Command genChoreoCommand(AutoPaths path) {

  //   ChoreoTrajectory pathTraj = Choreo.getTrajectory(path.getPath());
  //   drivetrain.setPose(pathTraj.getInitialPose());
  //   /* First put the drivetrain into auto run mode, then run the auto */
  //   var thetaController = new PIDController(5, 0, 0);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   Command swerveCommand = Choreo.choreoSwerveCommand(
  //       pathTraj, // Choreo trajectory from above
  //       drivetrain::getPose, // A function that returns the current field-relative pose of the robot: your
  //                            // wheel or vision odometry
  //       new PIDController(5, 0.0, 0.0), // PIDController for field-relative X
  //                                       // translation (input: X error in meters,
  //                                       // output: m/s).
  //       new PIDController(5, 0.0, 0.0), // PIDController for field-relative Y
  //                                       // translation (input: Y error in meters,
  //                                       // output: m/s).
  //       thetaController, // PID constants to correct for rotation
  //                        // error
  //       (ChassisSpeeds speeds) -> drivetrain
  //           .applyRequest(() -> forwardStraight.withVelocityX(-speeds.vxMetersPerSecond * MaxSpeed) // Drive forward
  //                                                                                                   // with
  //               // negative Y (forward)
  //               .withVelocityY(-speeds.vyMetersPerSecond * MaxSpeed) // Drive left with negative X (left)
  //               .withRotationalRate(-speeds.omegaRadiansPerSecond * MaxAngularRate) // Drive counterclockwise with
  //                                                                                   // negative X (left)
  //           ).ignoringDisable(true),
  //       () -> DriverStation.getAlliance().get().equals(Alliance.Red), // Whether or not to mirror the path based on
  //                                                                     // alliance (this assumes the path is created for
  //                                                                     // the blue alliance)
  //       drivetrain // The subsystem(s) to require, typically your drive subsystem only
  //   );

  //   return swerveCommand;

  // }
}
