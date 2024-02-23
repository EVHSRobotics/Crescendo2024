package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestDrive implements Subsystem{
    public TestDrive(){
        TalonFX fx = new TalonFX(2);
    }
}
