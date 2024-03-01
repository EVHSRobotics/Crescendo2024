// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SuperStructure.*;


public class Arm extends SubsystemBase {

  private TalonFX left;
  private TalonFX right;
  private CANcoder encoder;
  private PIDController rightPID;
  private Pigeon2 pigeonGyro;



  /** Creates a new Arm. */
  // Neg out back to base - for right and left
  // Pos out to talon fx arm - for right and left
  public Arm() {
    // -8 back to base
    // 5 to the talons
    // stragiht up is ~ 7
    // horizontal is ~ 0
    // down is ~ -1.6

    left = new TalonFX(43);
    right = new TalonFX(42);
    pigeonGyro = new Pigeon2(0, "drivetrain");
    
    rightPID = new PIDController(2, 0, 0);
    
    encoder = new CANcoder(61);

// encoder.set
    // left.set(ControlMode.Follower,)

    left.setControl(new Follower(42, true));

     TalonFXConfiguration motionMagicFXConfig = new TalonFXConfiguration();
         Slot0Configs configs = new Slot0Configs();
          configs.kS = 0;
          configs.kV = 0;
          configs.kA = 0;
          configs.kG = 0.2;
        
          configs.GravityType = GravityTypeValue.Arm_Cosine;
          configs.kP = 90;
          
          configs.kI = 0;
          configs.kD = 2.854;


      // motionMagicFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      // motionMagicFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // motionMagicFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = true;

        


    motionMagicFXConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motionMagicFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motionMagicFXConfig.MotionMagic.MotionMagicAcceleration = 1;
    motionMagicFXConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    motionMagicFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          motionMagicFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          // set accel and velocity for going up
          motionMagicFXConfig.withSlot0(configs);

          left.getConfigurator().apply(motionMagicFXConfig);
          right.getConfigurator().apply(motionMagicFXConfig);

      SmartDashboard.putNumber("KG", 0);
      SmartDashboard.updateValues();
  }

  public double getArmPosition() {
      return encoder.getAbsolutePosition().getValueAsDouble();
  }
  public boolean isArmInRange(ArmPosition pos) {
    return Math.abs(pos.getPos() - getArmPosition()) <= 0.03;
  }
  public void moveArm(double perOut) {
    right.setControl(new DutyCycleOut(-perOut));
  }
  public void reset(){
    right.setPosition(0);
  }

     
    public void resetSensor() {
        
    }

    public void setPosition(double position) {
      SmartDashboard.putNumber("setpos", position);
      SmartDashboard.updateValues();
      position += 0.0045;
        
      right.setControl(new MotionMagicVoltage(
        MathUtil.clamp(position, ArmPosition.REVERSE_TIPPING.getPos(), ArmPosition.LOW_INTAKE.getPos())
      ));

  }    

    public void setVoltage(double voltage) {
      SmartDashboard.putNumber("armVoltageFFChar", voltage);
      SmartDashboard.updateValues();
      // feedforward as needed
      right.setControl(new VoltageOut(-voltage));
    }


    public double getVelocity(){
      return right.getVelocity().getValueAsDouble();
        }
    
    public double getGyroPitch() {
      return pigeonGyro.getPitch().getValueAsDouble();
      
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
