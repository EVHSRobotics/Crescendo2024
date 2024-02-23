package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class GroundIntake extends Command {

    Intake intake;
    Arm arm;

    public GroundIntake(Arm arm, Intake intake){
        this.arm = arm;
        this.intake = intake;
        addRequirements(arm);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.arm.setPosition(-0.1);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        
        super.end(interrupted);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        this.intake.runIntake(0.7);
        super.execute();
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
