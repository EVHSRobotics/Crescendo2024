package frc.robot.autos;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SuperStructure.IntakeMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AdjustArmRpmFar extends Command {
        private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  
  
    private Arm arm;
    private Intake intake;
    private Shooter shoot;
    private Timer shootTimer;
   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                             
    private boolean isAutoFinished = false;
  
    /** Creates a new ShootNoteAuto. */
    public AdjustArmRpmFar(Arm arm, Intake intake, Shooter shoot) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.arm = arm;
      this.intake = intake;
      this.shoot = shoot;
  
      shootTimer = new Timer();
  
      addRequirements(arm);
      addRequirements(intake);
      addRequirements(shoot);
      
    }

    @Override
    public void initialize() 
    {
      shoot.motionMagicVelo(40);
      arm.setPosition(0.03);    
      isAutoFinished = false;
  
  
      shootTimer.purge();

      intake.pushIntake(0);
        
      // shootTimer.schedule(new TimerTask() {
      //   @Override
      //   public void run() {
  
      
      shootTimer.schedule(new TimerTask() {
        @Override
        public void run() {
          this.cancel();
          intake.pushIntake(IntakeMode.OUTTAKE.getSpeed());
          shootTimer.schedule(new TimerTask() {
  
            @Override
            public void run() {
              this.cancel();
              intake.pushIntake(0);
              shoot.motionMagicVelo(0);
              isAutoFinished = true;
            }
  
          }, 750);
        }
      }, 750);
    // }
    // }, 1000);
      
    }

    @Override
    public void execute(){
        
        shoot.motionMagicVelo(60);
        arm.setPosition(0.027);    
      }

    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAutoFinished;
  }
}
