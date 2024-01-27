package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    VictorSPX intake;
     private DigitalInput banner;
    public Intake(){
        intake = new VictorSPX(10);
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
