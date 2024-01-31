package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;
    XboxController controller;
    private boolean useAlgoShooting = false;
    public Shoot(Shooter shooter, XboxController controller){
        this.shooter = shooter;
        this.controller = controller;
    }

    public void initialize() {

    }
    public void execute() {
        SmartDashboard.putNumber("power", controller.getLeftX());
        SmartDashboard.updateValues();

            if (useAlgoShooting) {

        
                double output = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0);
                if (controller.getYButton()){
                    shooter.setShooterSpeed(output);
                }
            }
            else {

             shooter.setShooterSpeed(controller.getAButton() ? -0.35 : 0);
            }
    }
}
