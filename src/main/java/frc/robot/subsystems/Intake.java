package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.SuperStructure.ArmPosition;
import frc.robot.commands.SuperStructure.IntakeMode;

public class Intake implements Subsystem {
  // private final static ColorSensorV3 m_colorSensor = new
  // ColorSensorV3(I2C.Port.kOnboard);
  private DigitalInput bannerSensor;
  private TalonFX intake;
  public boolean useBanner;
  public boolean didSeeNote = false;
  public Timer noteReverseTimer = new Timer();
  

  public Intake() {
    useBanner = true;
    bannerSensor = new DigitalInput(1); // 1 is the one that works!
    intake = new TalonFX(44);

      TalonFXConfiguration configuration = new TalonFXConfiguration();
configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    
    intake.getConfigurator().apply(configuration);
  }

  public void keepNote() {
    if (!getBanner()) {
      intake.set(0.15);
    }
    else {
      intake.set(0);
    }
  } 
   public double getIntakeSpeed() {

    return intake.getVelocity().getValueAsDouble();
  }
  public void pushIntake(double power) {
   
    intake.set(power);
    
  }
  public void runIntake(double power) {
    if (useBanner) {
        // Outtaking is ok
        SmartDashboard.putNumber("test", power);
        if (!getBanner() || power < 0) {
                    didSeeNote = false;

          intake.set(power);
        } else {
          if (!didSeeNote) {
                     didSeeNote = true;

            noteReverseTimer.restart();

          }
          else {
            System.out.println(noteReverseTimer.get());
            if (noteReverseTimer.get() < 0.5) {

             
                intake.set(-0.075);
              

            }
            else {
              
             intake.set(0);

            }

          } 
          
        }


      
    } else {
      intake.set(power);
    }
    SmartDashboard.putNumber("BannerSeen?", power);
    SmartDashboard.updateValues();

  }

  public void intakePushOut (){
    intake.set(SmartDashboard.getNumber("IntakePushOutSpeed", -0.5));
  }

  public void shootAmp() {
    intake.set(-1);
  }

  public boolean getBanner() {

    return !bannerSensor.get();
  }

  public void setIntakeSpeed(double percentOutput) {
    intake.set(percentOutput);
  }
}
