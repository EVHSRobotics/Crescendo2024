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
  /** Creates a new Shoot. */
  public Shoot() {
    //1 is port for arm
     armController = new XboxController(1);
    // Use addRequirements() here to declare subsystem dependencies.

    shooter = new Shooter();
    
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  double output = NetworkTableInstance.getDefault().getTable("shootModel").getEntry("predictedPerOut").getDouble(0);
  if (armController.getYButton()){
    shooter.setShooterSpeed(output);
  }
  shooter.setShooterSpeed(output);
 
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
