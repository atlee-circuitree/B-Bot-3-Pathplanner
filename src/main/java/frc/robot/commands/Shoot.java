// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private Shooter shooter;
  private double targetVelocity;
  private double speed;
  /** Creates a new Shoot. */


  public Shoot(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(shooter);
  }
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.spin(speed);
  }

  public void execute() {

  }
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
