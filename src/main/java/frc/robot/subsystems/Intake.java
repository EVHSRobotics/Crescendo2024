package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem{
    VictorSPX intake;
    public Intake(){
        intake = new VictorSPX(10);
    }

    public void runIntake(double power){
        intake.set(ControlMode.PercentOutput, power);
    }
}
