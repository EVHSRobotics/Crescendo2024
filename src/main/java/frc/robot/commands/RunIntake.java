package frc.robot.commands;

import java.util.Timer;
import java.util.TimerTask;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command{

    Intake intake;
    XboxController controller;
    RunArm armDriver;
    double shootIntakeTime = 0;
    boolean shootAmp = false;

    public RunIntake(Intake intake, XboxController controller, RunArm armDriver){
        this.intake = intake;
        this.armDriver = armDriver;
        this.controller = controller;

        shootAmp = false;
        shootIntakeTime = 0;

        addRequirements(intake);
    }

    public void initialize() {

    }
    public void execute() {
        SmartDashboard.putNumber("powerother", controller.getLeftY());
        SmartDashboard.putNumber("pov", controller.getPOV());
        SmartDashboard.updateValues();

      
        




       
        // }
        // else {

        //     intake.runIntake(0);
        // }
    }


}
