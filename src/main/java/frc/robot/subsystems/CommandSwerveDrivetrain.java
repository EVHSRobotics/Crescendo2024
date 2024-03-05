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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            0.5,
                                            1,
                                            new ReplanningConfig(),
                                            0.004),
                                            () ->  DriverStation.getAlliance().get().equals(Alliance.Red),
                                            
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

     public PathPlannerAuto getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
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
        m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, pose);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public Command moveToHeading(double angle, Supplier<Double> suppX, Supplier<Double> suppY){
        SwerveRequest.FieldCentricFacingAngle heading = new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(Rotation2d.fromDegrees(angle)).withVelocityX(suppX.get() * 6).withVelocityY(suppY.get() * 6);
        heading.HeadingController.setPID(0.8, 0.0025, 0);
        heading.HeadingController.enableContinuousInput(-180, 180);
        return applyRequest(() -> heading);
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
}
