package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;
    XboxController controller;
    public Shoot(XboxController controller){
        shooter = new Shooter();
        this.controller = controller;
    }

    public void initialize() {

    }
    public void execute() {
        SmartDashboard.putNumber("power", controller.getLeftX());
        SmartDashboard.updateValues();
        shooter.setShooterSpeed(controller.getLeftY());
    }
}
