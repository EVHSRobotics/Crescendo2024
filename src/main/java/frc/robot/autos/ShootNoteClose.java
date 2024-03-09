public class ShootNoteClose extends Command {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  
  
    private Arm arm;
    private Intake intake;
    private Shooter shoot;
    private Timer shootTimer;
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                             
    private boolean isAutoFinished = false;
  
    /** Creates a new ShootNoteAuto. */
    public ShootNoteAuto(Arm arm, Intake intake, Shooter shoot) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.arm = arm;
      this.intake = intake;
      this.shoot = shoot;
  
      shootTimer = new Timer();
  
      addRequirements(arm);
      addRequirements(intake);
      addRequirements(shoot);
    }
}