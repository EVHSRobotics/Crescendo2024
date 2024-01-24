// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

  private Shooter shooter;
  private XboxController armController;
  private boolean useAlgoShooting = false;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, XboxController armController) {
    //1 is port for arm
     this.armController = armController;
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (useAlgoShooting) {

    
  double output = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0);
  if (armController.getYButton()){
    shooter.setShooterSpeed(output);
  }
}
else {

  shooter.setShooterSpeed(armController.getAButtonPressed() ? 1 : 0);
}
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
