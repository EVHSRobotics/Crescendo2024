// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.util.Ti/mer;
import java.util.TimerTask;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Leds.SparkLEDColors;
import pabeles.concurrency.IntOperatorTask.Max;

public class SuperStructure extends Command {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private Timer m_timer;
  public boolean bannerSeen = false;

  boolean robotCentricBool = false;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
   
   
      // driving in open loop
  
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

      
      private Supplier<Double> driveTrainSupplier;

    private Supplier<Double> driveTrainXSupplier;
  private Supplier<Double> driveTrainYSupplier;
private CommandXboxController controller;
  private Arm arm;
  private boolean algoShootBoolean = true;
  private Intake intake;
  private Leds ledSub;
  private Shooter shoot;
  private ArmPosition currentPosition = ArmPosition.STOW;
  private Climbers climbers;
  private IntakeMode currentIntake = IntakeMode.MANUAL;
  private boolean cancelAlgoShoot = false;

  private GenericEntry boardAlgoShoot;
  private GenericEntry boardBanner;
  private GenericEntry armPositionEntry;
  private GenericEntry intakeModeEntry;
  private GenericEntry flyWheelSpeedEntry;
  private GenericEntry armThetaEntry;
  private GenericEntry txEntry;
  private GenericEntry tyEntry;
  private GenericEntry taEntry;

  private double theta = 0.0;
  private double speedFly = 0.0;
  private XboxController operator;
  private Timer timer = new Timer();
  private java.util.Timer intakeTimer = new java.util.Timer();
  private java.util.Timer autoTippingTimer = new java.util.Timer();
  private XboxController driver;

  public enum IntakeMode {
    OUTTAKE(0.4, 1500), // for algo shoot and amp shoot
    INTAKE(0.5, 750), // for high intake shoot
    INTAKE_HIGH(0.25, 750),
    MANUAL(0, 1000),
    REVERSE(-0.2, 1000),
    INTAKE_AUTO(0.5, 1000);

    private double speed;
    private long time; // IN MS


    IntakeMode(double speed, long time) {
      this.speed = speed;
      this.time = time;
    }

    public double getSpeed() {
      return this.speed;
    }

    public long getTime() {
      return this.time;
    }
  }

  public enum ArmPosition {
    REVERSE_TIPPING(-0.3),
    STOW(-0.25),
    LOW_INTAKE(0.055),
    HIGH_INTAKE(-0.167),
    // 0.01 amp
    AMP(0.01),
    SHOOT(-0.02),
    STAGEFIT(0.01),
    ALGO(0),
    CLIMB(-0.43),
    FEEDER(0.05),
    HORIZONTAL(0);

    private double pos;

    ArmPosition(double pos) {
      this.pos = pos;
    }

    public double getPos() {
      return this.pos;
    }
  }

  /** Creates a new SuperStructure. */
  public SuperStructure(Arm arm, Intake intake, Shooter shoot, Climbers climbers, Leds led, XboxController driver,
      XboxController operator, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    this.shoot = shoot;
    this.ledSub = led;
    this.climbers = climbers;

    this.driver = driver;

    this.operator = operator;
    this.controller = controller;


    SmartDashboard.putNumber("offset", 0);
    SmartDashboard.updateValues();
   
    algoShootBoolean = true;
    driveTrainSupplier = () -> (Math.signum(driver.getRightX())
        * -(Math.abs(driver.getRightX()) > 0.15 ? Math.abs(Math.pow(driver.getRightX(), 2)) + 0.1 : 0))
        * MaxAngularRate;

    driveTrainXSupplier = () -> (Math.signum(driver.getLeftY())
                * -(Math.abs(driver.getLeftY()) > 0.1 ? Math.abs(Math.pow(driver.getLeftY(), 2)) + 0.1 : 0))
                * MaxSpeed;
      driveTrainYSupplier = () -> (Math.signum(driver.getLeftX())
                * -(Math.abs(driver.getLeftX()) > 0.1 ? Math.abs(Math.pow(driver.getLeftX(), 2)) + 0.1 : 0))
                * MaxSpeed;

    addRequirements(arm);
    addRequirements(intake);
    addRequirements(shoot);
    addRequirements(climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("setArm", 0);
    // SmartDashboard.putNumber("setRPM", 0);
    // SmartDashboard.updateValues();

    
    algoShootBoolean = true;
    speedFly = 0.0;
    theta = 0.0;
    currentPosition = ArmPosition.STOW;
    currentIntake = IntakeMode.MANUAL;
    ledSub.setLED(SparkLEDColors.RAINBOW);
    // boardAlgoShoot = Shuffleboard.getTab("DriverStation_2024").add("AlgoShoot", true).getEntry();
    // boardBanner = Shuffleboard.getTab("DriverStation_2024").add("Banner", false).getEntry();
    // armPositionEntry = Shuffleboard.getTab("DriverStation_2024").add("ArmPosition", "Stow").getEntry();
    // intakeModeEntry = Shuffleboard.getTab("DriverStation_2024").add("IntakeMode", "Manual").getEntry();
    // flyWheelSpeedEntry = Shuffleboard.getTab("DriverStation_2024").add("FlyWheelSpeed", 0.0).getEntry();
    // armThetaEntry = Shuffleboard.getTab("DriverStation_2024").add("ArmTheta", 0.0).getEntry();
    // tyEntry = Shuffleboard.getTab("DriverStation_2024").add("Limelight_TY", 0.0).getEntry();
    // txEntry = Shuffleboard.getTab("DriverStation_2024").add("Limelight_TX", 0.0).getEntry();
    // taEntry = Shuffleboard.getTab("DriverStation_2024").add("Limelight_TA", 0.0).getEntry();

    m_timer = new Timer();
    // Sets up Swerve
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(driveTrainXSupplier.get()) // Drive forward with
            // negative Y (forward)
            .withVelocityY(driveTrainYSupplier.get()) // Drive left with negative X (left)
            .withRotationalRate(driveTrainSupplier.get()) // Drive counterclockwise with negative X (left)
        ));


        controller.y().whileTrue(drivetrain.moveToHeading(58, driveTrainXSupplier, driveTrainYSupplier));
        controller.b().whileTrue(alignToAmp());
    

  }
 public Command alignToAmp() {
  Pose2d drivetrainPose = TunerConstants.DriveTrain.getPose();
  driveTrainXSupplier = () -> (((DriverStation.getAlliance().get() == Alliance.Red ? -6.45 : 6.45) -drivetrainPose.getX()) * 0.01);
  driveTrainYSupplier = () -> ((3.49-drivetrainPose.getY()) * 0.01);
    return drivetrain.moveToHeading(DriverStation.getAlliance().get() == Alliance.Red ? -90 : 90, driveTrainXSupplier, driveTrainYSupplier);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // arm.setPosition(SmartDashboard.getNumber("setArm", 0));
    // shoot.motionMagicVelo(SmartDashboard.getNumber("setRPM", 0));
    // intake.pushIntake(operator.getLeftY());
    SmartDashboard.putBoolean("Banner", intake.getBanner());
    SmartDashboard.updateValues();
    if (operator.getXButton() || operator.getLeftTriggerAxis() >= 0.3) {
      // Cancel button for algo shoot
      // if (currentPosition == ArmPosition.ALGO) {
        cancelAlgoShoot = true;
        setPosition(ArmPosition.STAGEFIT);
        setIntakeMode(IntakeMode.MANUAL);
        shoot.motionMagicVelo(0);

    }
    // We only want it to be set to false nto to true
    // if (intake.didSeeNote && intake.getBanner()) {
    //   intake.didSeeNote = false;
    // }

    // if (LimelightHelpers.getTX("limelight") != 0.0) {
    // Vision.lastTX = LimelightHelpers.getTX("limelight");

    // }
    // SmartDashboard.putNumber("posedX", TunerConstants.DriveTrain.getPigeon2());
        // SmartDashboard.putNumber("posedY", TunerConstants.DriveTrain.getPose().getRotation());

        SmartDashboard.putNumber("climbArm", arm.getArmPosition());
        SmartDashboard.putNumber("vx", drive.VelocityX);
                SmartDashboard.putNumber("vy", drive.VelocityY);

    SmartDashboard.putNumber("yaw",  Vision.lastTX
    );
        // SmartDashboard.putNumber("limelight", ( output));
        // SmartDashboard.putNumber("profiledOuputAlign", profiledOutput);
        SmartDashboard.updateValues();

    if (operator.getRightBumperPressed()) {
      cancelAlgoShoot = false;
      setPosition(ArmPosition.ALGO);
      ledSub.setLED(SparkLEDColors.ALGO_AIM);
      Vision.resetPIDController();
    } else if (operator.getRightBumper()) {
      if (MathUtil.applyDeadband(Math.abs(driver.getRightX()), 0.1) > 0) {
        driveTrainSupplier = () -> (Math.signum(driver.getRightX())
            * -(Math.abs(driver.getRightX()) > 0.15 ? Math.abs(Math.pow(driver.getRightX(), 2)) + 0.1 : 0))
            * MaxAngularRate;
      } else {
        driveTrainSupplier = () -> Vision.aimLimelightObject("limelight") * MaxAngularRate;

      }
      SmartDashboard.putNumber("predSpeed", speedFly);
      SmartDashboard.putNumber("velocity", shoot.getVelocity());
      if (MathUtil.applyDeadband(Vision.getLimelightAprilTagTXError(), 1.5
      ) != 0 || MathUtil.applyDeadband(speedFly - shoot.getVelocity(), 5) != 0) {
        operator.setRumble(RumbleType.kBothRumble, 0.5);

      } else {
        operator.setRumble(RumbleType.kBothRumble, 0);

      }

    }

    else if (operator.getRightBumperReleased()) {
      // Resets rotation back to driver control
      driveTrainSupplier = () -> (Math.signum(driver.getRightX())
          * -(Math.abs(driver.getRightX()) > 0.15 ? Math.abs(Math.pow(driver.getRightX(), 2)) + 0.1 : 0))
          * MaxAngularRate;
      if (!cancelAlgoShoot) {
        setIntakeMode(IntakeMode.OUTTAKE);
        ledSub.setLED(SparkLEDColors.ALGO_SHOOT);

      }

    }
     
    else if (operator.getAButton()) {

      setPosition(ArmPosition.LOW_INTAKE);
      setIntakeMode(IntakeMode.INTAKE);
      ledSub.setLED(SparkLEDColors.LOW_INTAKE);
     /* if (intake.getBanner()) {
        bannerSeen = true;
      }
      if (bannerSeen) {
        if (m_timer.get() < 1) {
          intake.intakePushOut();
        }

      } */

    }

    // else if (operator.getAButtonReleased()) {
    //   m_timer.reset();
    //   bannerSeen = false;
    // } 

    
    else if (operator.getLeftBumper()) {
cancelAlgoShoot = false;
      setPosition(ArmPosition.AMP);
      ledSub.setLED(SparkLEDColors.AMP);

    }
   
    else if (operator.getLeftBumperReleased()) {
            if (!cancelAlgoShoot) {

             setIntakeMode(IntakeMode.OUTTAKE);
            }
    }    
    else if (operator.getYButton()) {
      setPosition(ArmPosition.STOW);
      setIntakeMode(IntakeMode.MANUAL);
      ledSub.setLED(SparkLEDColors.RAINBOW);

    } 
    else if (operator.getStartButton()) {
      cancelAlgoShoot = false;

      setPosition(ArmPosition.FEEDER);
    }
    else if (operator.getStartButtonReleased()) {
      if (!cancelAlgoShoot) {
        setIntakeMode(IntakeMode.OUTTAKE);
      }
    }
    else if (operator.getBButton()) {
      setPosition(ArmPosition.HIGH_INTAKE); 
      setIntakeMode(IntakeMode.INTAKE_HIGH);
      ledSub.setLED(SparkLEDColors.HIGH_INTAKE);

    } 
    
    else if (operator.getPOV() == 270) {
      setIntakeMode(IntakeMode.MANUAL);
      setPosition(ArmPosition.STAGEFIT);

    }
    else if (operator.getPOV() == 0) {
      setIntakeMode(IntakeMode.MANUAL);
      setPosition(ArmPosition.CLIMB);
    }
    else if (MathUtil.applyDeadband(operator.getLeftY(), 0.1) != 0) {
      // Override Intake mode at any point to be manual
      currentIntake = IntakeMode.MANUAL;
    } 
    
    else {
      // Should be running continously
      // Balancing algo
      // autoBalancingAlgo();
      operator.setRumble(RumbleType.kBothRumble, 0);

      driveTrainXSupplier = () -> (Math.signum(driver.getLeftY())
      * -(Math.abs(driver.getLeftY()) > 0.1 ? Math.abs(Math.pow(driver.getLeftY(), 2)) + 0.1 : 0))
      * MaxSpeed;
driveTrainYSupplier = () -> (Math.signum(driver.getLeftX())
      * -(Math.abs(driver.getLeftX()) > 0.1 ? Math.abs(Math.pow(driver.getLeftX(), 2)) + 0.1 : 0))
      * MaxSpeed;

        driveTrainSupplier = () -> (Math.signum(driver.getRightX())
          * -(Math.abs(driver.getRightX()) > 0.15 ? Math.abs(Math.pow(driver.getRightX(), 2)) + 0.1 : 0))
          * MaxAngularRate;
    
    }
  //   if (driver.getYButton()) {

  // // drivetrain.generatePathAmp().execute();
  //     drivetrain.moveToHeading(58, driveTrainXSupplier, driveTrainYSupplier);
  //   }
  //   else if (driver.getBButton()) {
  //     // drivetrain.generatePathSpeaker().execute();
  //       drivetrain.moveToHeading(270, driveTrainXSupplier, driveTrainYSupplier);

  //   }

  if (driver.getPOV() == 180) {
    algoShootBoolean = !algoShootBoolean;
  }

  if (driver.getRightTriggerAxis() > 0.1) {
    
    climbers.moveClimbers(driver.getRightTriggerAxis());
  }
  else if (driver.getLeftTriggerAxis() > 0.1) {
    climbers.moveClimbers(-driver.getLeftTriggerAxis());
  }
  else {
    climbers.moveClimbers(0);
  }

  
    // if (driver.getXButton()) {
      
      // double y = (Vision.aimLimelightObject("limelight-intake") *
      //   MaxAngularRate);
      //   SmartDashboard.putNumber("limelight intake", Vision.aimLimelightObject("limelight-intake") *
      //   MaxSpeed * 0.5);
      //   SmartDashboard.updateValues();
      //   double x = Math.signum(driver.getLeftY())
      //           * -(Math.abs(driver.getLeftY()) > 0.1 ? Math.abs(Math.pow(driver.getLeftY(), 2)) + 0.1 : 0)
      //           * MaxSpeed;
              
      //   drivetrain.applyRequest(() -> robotCentric
      //   .withVelocityX(x)
      //   .withRotationalRate(y)).execute();
    // }
    // else if (driver.getRightBumper()) {
    //   Vision.caliOffset -= 0.01;
    // }
    // else if (driver.getLeftBumper()) {
    //   Vision.caliOffset += 0.01;
    // }

    SmartDashboard.putNumber("Calibration Offset", Vision.caliOffset);
    SmartDashboard.updateValues();
  
    // }
    // else {
    // driveTrainSupplier = () -> (Math.signum(driver.getRightX())
    // * -(Math.abs(driver.getRightX()) > 0.15 ?
    // Math.abs(Math.pow(driver.getRightX(), 2)) + 0.1 : 0))
    // * MaxAngularRate;
    // }

    if (operator.getPOV() == 180) {
      intake.useBanner = !intake.useBanner;
    }

    if (currentPosition == ArmPosition.ALGO) { // removed && operator.rightBumper()
      if (algoShootBoolean) {

        double tempTheta = Vision.getPredTheta();
        double tempSpeed = Vision.getPredVelocity();

        // double tempSpeed = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut")
        //     .getDouble(0);
        // double tempTheta = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta")
        //     .getDouble(0);

        if (LimelightHelpers.getTV("limelight")) {
          theta = tempTheta;
          speedFly = tempSpeed;
        }
        
        
        shoot.motionMagicVelo(tempSpeed);
      
        arm.setPosition(theta);
          // arm.setPosition(drive.VelocityX, theta);
        
      }
      else {
        theta = 0.03;
        speedFly = 60;
        shoot.motionMagicVelo(speedFly);
        arm.setPosition(theta);

        // arm.setPosition(drive.VelocityX, theta);
      }
    } 

    else {
      if (currentPosition == ArmPosition.AMP) {
        // shoot.motionMagicVelo(-0.185*LimelightHelpers.getTY("limelight") +20.8);
        shoot.motionMagicVelo(20, 10);

      }
     
      else if (currentPosition == ArmPosition.FEEDER) {
        shoot.motionMagicVelo(70);
      }
        else if (currentPosition == ArmPosition.HIGH_INTAKE) {
      // shoot.motionMagicVelo(-20+);  01
      if(intake.getBanner()){
        // setPosition(ArmPosition.STAGEFIT);
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);

      }else {
        operator.setRumble(RumbleType.kBothRumble, 0.3);
        driver.setRumble(RumbleType.kBothRumble, 0.3);
      }
    }
     else {
      // System.out.println("HELLOE");
      shoot.stopShooters();
     }

     if (currentPosition == ArmPosition.CLIMB) {
arm.setPositionClimb();
     }
    
     else {

     
      arm.setPosition(currentPosition.getPos());
     }
      // arm.setPosition(arm.getArmPosition() -
      // MathUtil.applyDeadband(operator.getLeftY() / 10, 0.005));

    }
    if (currentPosition == ArmPosition.LOW_INTAKE) {
      if(intake.getBanner()){
        // setPosition(ArmPosition.STAGEFIT);
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);

      }else {
        setPosition(ArmPosition.LOW_INTAKE);
        operator.setRumble(RumbleType.kBothRumble, 0.3);
        driver.setRumble(RumbleType.kBothRumble, 0.3);
      }
      shoot.stopShooters();
    }
    // SmartDashboard.putNumber("pitch gyro", arm.getGyroPitch());

    // SmartDashboard.putNumber("ty", LimelightHelpers.getTY("limelight"));
    // SmartDashboard.putNumber("Algo shoot Output",
    //     NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
    // // shoot.motionMagicVelo(
    // // );
    // SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV("limelight"));

    // SmartDashboard.putBoolean("bnanner", intake.getBanner());
    // SmartDashboard.putNumber("Algo shoot theta",
    //     NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(0));
    // SmartDashboard.updateValues();
  

    if (currentIntake == IntakeMode.MANUAL) {
      // intake.runIntake(MathUtil.applyDeadband(operator.getRightY(), 0.1));
      // if (currentPosition == ArmPosition.STAGEFIT) {
      //   intake.runIntake(MaxAngularRate);
      // }
      // else {
        if (currentPosition == ArmPosition.STAGEFIT) {
          intake.keepNote();
        }
        else {
          intake.pushIntake(MathUtil.applyDeadband(operator.getLeftY() * -0.8, 0.1));
      }
      if (currentPosition != ArmPosition.ALGO) {
        operator.setRumble(RumbleType.kBothRumble, 0);
      }
      driver.setRumble(RumbleType.kBothRumble, 0);

    }
    else if (currentIntake == IntakeMode.INTAKE || currentIntake == IntakeMode.REVERSE || currentIntake == IntakeMode.INTAKE_HIGH) {

      intake.runIntake(currentIntake.getSpeed());
     

    } 
    else if (currentIntake == IntakeMode.OUTTAKE) {

      intake.pushIntake(currentIntake.getSpeed());
    }

    // boardAlgoShoot.setBoolean(algoShootBoolean);
    // boardBanner.setBoolean(intake.getBanner());
    // flyWheelSpeedEntry.setDouble(shoot.getVelocity());
    // taEntry.setDouble(LimelightHelpers.getTA("limelight"));
    // txEntry.setDouble(LimelightHelpers.getTX("limelight"));
    // tyEntry.setDouble(LimelightHelpers.getTY("limelight"));
    // armThetaEntry.setDouble(arm.getArmPosition());
    // switch (currentPosition) {
    //   case ALGO:
    //       armPositionEntry.setString("ALGO");

    //     break;
    //   case AMP:
    //       armPositionEntry.setString("AMP");

    //     break;
    //   case HIGH_INTAKE:
    //       armPositionEntry.setString("HIGH_INTAKE");

    //     break;
    //   case HORIZONTAL:
    //       armPositionEntry.setString("HORIZONTAL");

    //     break;
    //   case LOW_INTAKE:
    //       armPositionEntry.setString("LOW_INTAKE");

    //     break;
    //   case REVERSE_TIPPING:
    //       armPositionEntry.setString("REVERSE_TIPPING");

    //     break;
    //   case SHOOT:
    //       armPositionEntry.setString("SHOOT");

    //     break;
    //   case STAGEFIT:
    //       armPositionEntry.setString("STAGEFIT");

    //     break;
    //   case STOW:
    //       armPositionEntry.setString("STOW");

    //     break;
    //   default:
    //     break;

    // }

    // switch (currentIntake) {
    //   case INTAKE:
    //     intakeModeEntry.setString("INTAKE");
    //     break;
    //   case MANUAL:
    //     intakeModeEntry.setString("MANUAL");
    //     break;
    //   case OUTTAKE:
    //   intakeModeEntry.setString("OUTTAKE");
    //     break;
    //   case REVERSE:
    //   intakeModeEntry.setString("REVERSE");
    //     break;
    //   default:
    //     break;
      
    // }

    // Shuffleboard.update();


  }

  public void setPosition(ArmPosition pos) {
    this.currentPosition = pos;
  }

  public void setIntakeMode(IntakeMode mode) {

    // If mode isn't manual
    if (mode != IntakeMode.MANUAL) {
      // if the arm is in algo mode
      if (currentPosition == ArmPosition.ALGO || currentPosition == ArmPosition.AMP || currentPosition == ArmPosition.FEEDER) {
        // We can immediately shoot it
        currentIntake = mode;
        intakeTimer.purge();
        intakeTimer.schedule(new TimerTask() {
          @Override
          public void run() {
            // Sets intake mode back to manual
            currentIntake = IntakeMode.MANUAL;
            setPosition(ArmPosition.STAGEFIT); // Moves it to Stow
            ledSub.setLED(SparkLEDColors.RAINBOW);
            this.cancel();
          }
        }, mode.getTime());
      } 
      else {
        // This is if it is not in algo mode
        // First continously check if the arm is at the setpoint
        // intakeTimer.cancel();
      currentIntake = mode;
         
      }
    } else {
      this.currentIntake = mode;
    }
  }

  public void autoBalancingAlgo() {
    double pitch = arm.getGyroPitch();
    // SmartDashboard.putNumber("pitchitypitch", pitch);
    // SmartDashboard.updateValues();
    if (Math.abs(pitch) > 25) {
      currentPosition = Math.signum(pitch) > 0 ? ArmPosition.REVERSE_TIPPING : ArmPosition.HIGH_INTAKE;
    }
  }

  public void autoPickBackUpAlgo() {

    currentPosition = ArmPosition.STOW;
    autoTippingTimer.purge();

    autoTippingTimer.scheduleAtFixedRate(new TimerTask() {
      @Override
      public void run() {
        if (arm.isArmInRange(currentPosition)) {
          // Once it is, we can cancel this timer and set the currentIntake mode to the
          // specified mode, and schedule
          // a new comman that is supposed to run the intake till the time expires, or
          // banner sensor is triggered

          this.cancel();

          double pitch = arm.getGyroPitch();
          if (pitch > 0) {
            currentPosition = ArmPosition.LOW_INTAKE;
          } else {
            currentPosition = ArmPosition.REVERSE_TIPPING;
          }

          this.cancel();

        }
      }
    }, 0, 1);

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
