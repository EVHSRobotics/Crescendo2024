package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command{

    Intake intake;
    XboxController controller;
    public RunIntake(Intake intake, XboxController controller){
        this.intake = intake;
        this.controller = controller;
    }

    public void initialize() {

    }
    public void execute() {
        SmartDashboard.putNumber("powerother", controller.getRightY());
        SmartDashboard.updateValues();
        intake.runIntake(-controller.getLeftY());
    }


}
