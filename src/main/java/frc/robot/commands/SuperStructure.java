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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Leds.SparkLEDColors;

public class SuperStructure extends Command {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private Timer m_timer;
  public boolean bannerSeen = false;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private Supplier<Double> driveTrainSupplier;

    private Supplier<Double> driveTrainXSupplier;
  private Supplier<Double> driveTrainYSupplier;

  private Arm arm;
  private Intake intake;
  private Leds ledSub;
  private Shooter shoot;
  private ArmPosition currentPosition = ArmPosition.STOW;
  private IntakeMode currentIntake = IntakeMode.MANUAL;
  private boolean cancelAlgoShoot = false;

  private double theta = 0.0;
  private double speedFly = 0.0;
  private XboxController operator;
  private Timer timer = new Timer();
  private java.util.Timer intakeTimer = new java.util.Timer();
  private java.util.Timer autoTippingTimer = new java.util.Timer();
  private XboxController driver;

  public enum IntakeMode {
    OUTTAKE(1, 1000), // for algo shoot and amp shoot
    INTAKE(1, 750), // for high intake shoot
    MANUAL(0, 1000),
    REVERSE(-0.4, 1000);

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
    LOW_INTAKE(0.06),
    HIGH_INTAKE(-0.167),
    // 0.01 amp
    AMP(0.01),
    SHOOT(-0.02),
    STAGEFIT(0.01),
    ALGO(0),
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
  public SuperStructure(Arm arm, Intake intake, Shooter shoot, Leds led, XboxController driver,
      XboxController operator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    this.shoot = shoot;
    this.ledSub = led;

    this.driver = driver;

    this.operator = operator;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("setArm", 0);
    // SmartDashboard.putNumber("setRPM", 0);
    // SmartDashboard.updateValues();

    speedFly = 0.0;
    theta = 0.0;
    currentPosition = ArmPosition.STOW;
    currentIntake = IntakeMode.MANUAL;
    ledSub.setLED(SparkLEDColors.RAINBOW);
m_timer = new Timer();
    // Sets up Swerve
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(driveTrainXSupplier.get()) // Drive forward with
            // negative Y (forward)
            .withVelocityY(driveTrainYSupplier.get()) // Drive left with negative X (left)
            .withRotationalRate(driveTrainSupplier.get()) // Drive counterclockwise with negative X (left)
        ));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // arm.setPosition(SmartDashboard.getNumber("setArm", 0));
    // shoot.motionMagicVelo(SmartDashboard.getNumber("setRPM", 0));
    // intake.pushIntake(operator.getLeftY());
    if (operator.getXButton()) {
      // Cancel button for algo shoot
      // if (currentPosition == ArmPosition.ALGO) {
        cancelAlgoShoot = true;
        setPosition(ArmPosition.STAGEFIT);
        setIntakeMode(IntakeMode.MANUAL);
        shoot.motionMagicVelo(0);

      // setPosition(ArmPosition.STOW);
      // }
    }
    // We only want it to be set to false nto to true
    // if (intake.didSeeNote && intake.getBanner()) {
    //   intake.didSeeNote = false;
    // }
      
    SmartDashboard.putBoolean("bannerNEW", intake.getBanner());
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
      if (MathUtil.applyDeadband(Vision.getLimelightAprilTagTXError(), 3) != 0 || MathUtil.applyDeadband(speedFly - shoot.getVelocity(), 5) != 0) {
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

    } else if (operator.getAButton()) {

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

    else if (operator.getStartButton()) {
      // Tipping algo
      // autoPickBackUpAlgo();
    }
    
    else if (operator.getBButton()) {
      setPosition(ArmPosition.AMP);
      ledSub.setLED(SparkLEDColors.AMP);

    }
    
    else if (operator.getYButton()) {
      setPosition(ArmPosition.STOW);
      setIntakeMode(IntakeMode.MANUAL);
      ledSub.setLED(SparkLEDColors.RAINBOW);

    } 
    
    else if (operator.getLeftBumper()) {
      setPosition(ArmPosition.HIGH_INTAKE);
      setIntakeMode(IntakeMode.INTAKE);
      ledSub.setLED(SparkLEDColors.HIGH_INTAKE);

    } 
    
    else if (operator.getPOV() == 270) {
      setIntakeMode(IntakeMode.MANUAL);
      setPosition(ArmPosition.STAGEFIT);

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

    
        driveTrainSupplier = () -> (Math.signum(driver.getRightX())
          * -(Math.abs(driver.getRightX()) > 0.15 ? Math.abs(Math.pow(driver.getRightX(), 2)) + 0.1 : 0))
          * MaxAngularRate;
    
    }
if (driver.getYButton()) {
      drivetrain.moveToHeading(58, driveTrainXSupplier, driveTrainYSupplier).execute();;
    }
    else if (driver.getBButton()) {
            drivetrain.moveToHeading(270, driveTrainXSupplier, driveTrainYSupplier).execute();

    }
    if (driver.getXButton()) {
         driveTrainXSupplier = () -> (Vision.getObjectDistanceOutputVert())
                * MaxSpeed * 0.5;
      driveTrainYSupplier = () -> (Vision.aimLimelightObject("limelight-intake") *
        MaxSpeed * 0.5);
    }
    
    else {
      driveTrainXSupplier = () -> (Math.signum(driver.getLeftY())
                * -(Math.abs(driver.getLeftY()) > 0.1 ? Math.abs(Math.pow(driver.getLeftY(), 2)) + 0.1 : 0))
                * MaxSpeed;
      driveTrainYSupplier = () -> (Math.signum(driver.getLeftX())
                * -(Math.abs(driver.getLeftX()) > 0.1 ? Math.abs(Math.pow(driver.getLeftX(), 2)) + 0.1 : 0))
                * MaxSpeed;
    }
   
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

      double tempSpeed = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut")
          .getDouble(0);
      double tempTheta = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta")
          .getDouble(0);

      if (LimelightHelpers.getTV("limelight")) {
        theta = tempTheta;
        speedFly = tempSpeed;
      }
      shoot.motionMagicVelo(tempSpeed);
      arm.setPosition(theta);
    } 

    else {
      if (currentPosition == ArmPosition.AMP) {
        // shoot.motionMagicVelo(-0.185*LimelightHelpers.getTY("limelight") +20.8);
        shoot.motionMagicVelo(25, 13);

      }
        else if (currentPosition == ArmPosition.HIGH_INTAKE || currentPosition == ArmPosition.LOW_INTAKE) {
      // shoot.motionMagicVelo(-20+);
    }
     else {
      // System.out.println("HELLOE");
      shoot.stopShooters();
     }

      arm.setPosition(currentPosition.getPos());
      // arm.setPosition(arm.getArmPosition() -
      // MathUtil.applyDeadband(operator.getLeftY() / 10, 0.005));

    }
    SmartDashboard.putNumber("pitch gyro", arm.getGyroPitch());

    SmartDashboard.putNumber("ty", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("Algo shoot Output",
        NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0));
    // shoot.motionMagicVelo(
    // );
    SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV("limelight"));

    SmartDashboard.putBoolean("bnanner", intake.getBanner());
    SmartDashboard.putNumber("Algo shoot theta",
        NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedTheta").getDouble(0));
    SmartDashboard.updateValues();

    if (currentIntake == IntakeMode.MANUAL) {
      // intake.runIntake(MathUtil.applyDeadband(operator.getRightY(), 0.1));
      intake.pushIntake(operator.getLeftY() * 0.8);

    }
    else if (currentIntake == IntakeMode.INTAKE || currentIntake == IntakeMode.REVERSE) {

      intake.runIntake(currentIntake.getSpeed());

    } else if (currentIntake == IntakeMode.OUTTAKE) {

      intake.pushIntake(currentIntake.getSpeed());
    }
  }

  public void setPosition(ArmPosition pos) {
    this.currentPosition = pos;
  }

  public void setIntakeMode(IntakeMode mode) {

    // If mode isn't manual
    if (mode != IntakeMode.MANUAL) {
      // if the arm is in algo mode
      if (currentPosition == ArmPosition.ALGO) {
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
      } else {
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
    SmartDashboard.putNumber("pitchitypitch", pitch);
    SmartDashboard.updateValues();
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
