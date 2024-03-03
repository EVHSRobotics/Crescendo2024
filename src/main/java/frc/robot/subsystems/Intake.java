package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.SuperStructure.IntakeMode;

public class Intake implements Subsystem {
  // private final static ColorSensorV3 m_colorSensor = new
  // ColorSensorV3(I2C.Port.kOnboard);
  private DigitalInput bannerSensor;
  private TalonSRX intake;
  public boolean useBanner;

  public Intake() {
    useBanner = true;
    bannerSensor = new DigitalInput(1); // 1 is the one that works!
    intake = new TalonSRX(44);

  }
  public double getIntakeSpeed() {

    return intake.getSelectedSensorVelocity();
  }
  public void pushIntake(double power) {
   
    intake.set(ControlMode.PercentOutput, power);
    
  }
  public void runIntake(double power) {
    if (useBanner) {
        // Outtaking is ok
        SmartDashboard.putNumber("test", power);
        if (!getBanner() || power < 0) {
          intake.set(ControlMode.PercentOutput, power);
        } else {

          intake.set(ControlMode.PercentOutput, 0);

          
        }


      
    } else {
      intake.set(ControlMode.PercentOutput, power);
    }
    SmartDashboard.putNumber("BannerSeen?", power);
    SmartDashboard.updateValues();

  }

  public void intakePushOut (){
    intake.set(ControlMode.PercentOutput, SmartDashboard.getNumber("IntakePushOutSpeed", -0.5));
  }

  public void shootAmp() {
    intake.set(ControlMode.PercentOutput, -1);
  }

  public boolean getBanner() {

    return !bannerSensor.get();
  }

  public void setIntakeSpeed(double percentOutput) {
    intake.set(ControlMode.PercentOutput, percentOutput);
  }
}
