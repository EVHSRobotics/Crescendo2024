package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
  public boolean seenNote = false;
  public Timer timer = new Timer();
  

  public Intake() {
    useBanner = true;
    bannerSensor = new DigitalInput(1); // 1 is the one that works!
    intake = new TalonFX(44);

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // // var configs = configuration.Slot0;
    // // configs.kS = 0;
    // // configs.kV = 0;
    // // configs.kA = 0;
  
    // // configs.kP = 0.3;    
    // // configs.kI = 0;
    // // configs.kD = 0;
    // // var motionMagic = configuration.MotionMagic;

    
    // // motionMagic.MotionMagicAcceleration = 200;
    // // motionMagic.MotionMagicJerk = 2000;
    
    intake.getConfigurator().apply(configuration);
  }

  public Double topGetVelocity(){
    return intake.getVelocity().getValueAsDouble();  
  }

  public void topVoltageConsumer(Double voltage){
    SmartDashboard.putNumber("shooterVoltageFFChar", voltage);
    SmartDashboard.updateValues();
    intake.setControl(new VoltageOut(voltage));
  }

  public void setRPS(double RPS){
    intake.setControl(new MotionMagicVelocityVoltage(RPS));
  }

  public void keepNote() {
    if (!getBanner()) {
      intake.set(0.2);
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
    // first chceck if the sensor has seen the note, if false, and the banner is not detecting then intake, reset timer at this moment
    if(!seenNote && !getBanner()){
       intake.set(power);
       seenNote = getBanner();
       timer.reset();
    }

    //e we have already seen the note, and the timer is less thatn 1.5 seconds, we do a little rieverse itnake to push it back (might not be needed)
    else if (seenNote && (timer.get() <1.5)){
        intake.set(-0.1);
    }

    //since the SeenNote, would be true, and so would Banner, reset SeenNote as already stored. and the first if won't run as getBanner is still true, and neither will the second if run as timer has run out
    else {
      seenNote=false;

      intake.set(0);
    }
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
