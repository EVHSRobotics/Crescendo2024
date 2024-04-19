package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;

public class CheckNoteSecondLeft extends Command{

    Intake intake;
    boolean finished = false;
    public CheckNoteSecondLeft(Intake intake){
        this.intake = intake;
    }
    
    @Override
    public void initialize(){
        finished = false;

        if(!intake.getBanner()){
            CommandScheduler.getInstance().cancelAll();
            TunerConstants.DriveTrain.getAutoPath("Adjust_Second_Left").schedule();
        }
        finished = true;
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
