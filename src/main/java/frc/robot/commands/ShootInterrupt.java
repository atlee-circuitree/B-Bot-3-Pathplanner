// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootInterrupt extends Command {

  private Shooter shoot;
  private double speed;
  
  /** Creates a new ShootInterrupt. */
  public ShootInterrupt(Shooter shoot, double speed) {
    this.speed = speed;
    this.shoot = shoot;
    addRequirements(shoot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoot.spin(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
   {
    shoot.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
