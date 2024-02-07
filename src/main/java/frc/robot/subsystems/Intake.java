package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    TalonSRX intake;
     private DigitalInput banner;
    public Intake(){
        intake = new TalonSRX(44);
    }

    public void runIntake(double power){
        intake.set(ControlMode.PercentOutput, power);
    }
    public boolean getBanner() {
        return banner.get();
      }
      public void setIntakeSpeed(double percentOutput) {
        intake.set(ControlMode.PercentOutput, percentOutput);
      } 
 
}
