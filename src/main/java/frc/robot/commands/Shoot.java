package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;
    XboxController controller;
    private boolean useAlgoShooting = true;
    public Shoot(Shooter shooter, XboxController controller){
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(shooter);
    }

    public void initialize() {

    }
    public void execute() {
        // SmartDashboard.putNumber("power", controller.getRightTriggerAxis());
        // SmartDashboard.putNumber("rpmTop", shooter.getVelocity());
        // SmartDashboard.putNumber("shooterRPM", 0);
        SmartDashboard.updateValues();

            if (useAlgoShooting) {


                // double output//// = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0);
                if (controller.getYButton()){
                    shooter.motionMagicVelo(90);
                } else if (controller.getAButton()){
                    shooter.motionMagicVelo(50);
                }
                 else{
                    shooter.motionMagicVelo(0);
                 }
            }
            else {
                // if(controller.getLeftY() > -0.01)
                    // shooter.setShooterRPM(SmartDashboard.getNumber("rpmTop", 0));
                //  else{
                //     shooter.setRPS(0);

                // }

                // shooter.setShooterSpeed(controller.getRightY());

            //  shooter.setShooterSpeed(controller.getAButton() ? -1 : 0);
            }
    }
}
