package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ParentConfigurator;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.generated.SwerveVoltageRequest;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.FieldCentricFacingAngle heading = new SwerveRequest.FieldCentricFacingAngle();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePP();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        optimize();

        
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePP();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        optimize();
    }

    private void optimize(){
        // m_pigeon2.optimizeBusUtilization();
        // for(SwerveModule module : this.Modules){
        //     module.getCANcoder().
        // }
    }

    private void configurePP(){

        AutoBuilder.configureHolonomic(
            this::getPose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(4, 0, 0),
                                            new PIDConstants(4, 0, 0),
                                            0.5,
                                            1,

                                            new ReplanningConfig(),
                                            0.004),
                                            () ->  DriverStation.getAlliance().get().equals(Alliance.Red),
                                            
            this); // Subsystem for requirements

                    // Pathfinding.setPathfinder(new LocalADStar());

    this.heading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    this.heading.HeadingController.setPID(10, 0.5, 2);


    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command applyRequestOnce(Supplier<SwerveRequest> requestSupplier) {
        return runOnce(() -> this.setControl(requestSupplier.get()));
    }

     public PathPlannerAuto getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }
    public Command generatePathSource() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        // Pathfinding.setStartPosition(getPose().getTranslation());
        Pose2d targetPose = new Pose2d(15.26, 1.41, Rotation2d.fromDegrees(300.297));
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        // Pathfinding.setDynamicObstacles(null, this.getPose().getTranslation());

        return pathfindingCommand;

    }
    public Command generatePathSpeaker() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        
        Pathfinding.setStartPosition(getPose().getTranslation());

        Pose2d targetPose = new Pose2d(2, 5.52, Rotation2d.fromDegrees(180.02334));
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

                
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;

    }
    public Command generatePathAmp() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(1.82, 7.30, Rotation2d.fromDegrees(91.50136));
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        return pathfindingCommand;

    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setPose(Pose2d pose){
        m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue() + pose.getRotation().getDegrees()), m_modulePositions, pose);
    }

    public void setPoseOffset(Pose2d pose){
        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, pose);
    }   


    
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

   
    public Command moveToHeading(double angle, Supplier<Double> suppX, Supplier<Double> suppY){
        // heading.HeadingController.setPID(2, 0.0, 0);
        
        // heading.HeadingController.enableContinuousInput(0, 360);
        // heading.RotationalDeadband = 5;
        // heading.Deadband = 5;


       return applyRequest(() -> heading
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withTargetDirection(Rotation2d.fromDegrees(angle))
            .withVelocityX(suppX.get())
            .withVelocityY(suppY.get()));

    }

    public void driveWithVoltage(double Voltage){
       applyRequest(() -> new SwerveVoltageRequest().withVoltage(Voltage)).execute();
    }

    public void steerWithVoltage(double Voltage){
       applyRequest(() -> new SwerveVoltageRequest(false).withVoltage(Voltage)).execute();
    }
    
    public double getVeloDrive(){
        double sum = 0;
        for(SwerveModule module : Modules){
            sum += module.getDriveMotor().getVelocity().getValueAsDouble();
        }
        return sum/4;
    }

    public double getVeloSteer(){
        double sum = 0;
        for(SwerveModule module : Modules){
            sum += module.getCANcoder().getVelocity().getValueAsDouble();
        }
        return sum/4;

    }

    public double getRate(){
        return m_pigeon2.getRate();
    }

}
