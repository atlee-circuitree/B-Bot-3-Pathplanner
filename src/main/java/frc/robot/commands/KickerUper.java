// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.KickUp;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KickerUper extends Command {
  private final KickUp kickUp;
  private Shooter shoot;
   
    /** Creates a new KickerUper. */
    public KickerUper(KickUp kickUp, Shooter shoot) { 
      this.kickUp = kickUp;
      this.shoot = shoot;
    addRequirements(kickUp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //kickUp.shoot();

    // Doug Testing
    // It causes the kicker to start instantly before checking speed.
    kickUp.stop();
  }

  @Override
  public void execute() {
    if (shoot.isAtSpeed()) {
      kickUp.shoot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    kickUp.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
