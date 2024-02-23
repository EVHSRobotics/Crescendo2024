package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
  //private final static ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private TalonSRX intake;
    private DigitalInput bannerintake ;
    private boolean bannerseen;
    public Intake(){
         bannerintake = new DigitalInput(2);
          intake = new TalonSRX(44);
        
    }

    public void runIntake(double power){
      
        intake.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("BannerSeen?", power);
        SmartDashboard.updateValues();
      }
    
    public boolean getBanner() {
        return bannerintake.get();
      }
      public void setIntakeSpeed(double percentOutput) {
        intake.set(ControlMode.PercentOutput, percentOutput);
      } 
    }
  
