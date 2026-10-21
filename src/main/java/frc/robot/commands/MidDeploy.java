// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MidDeploy extends Command {
   private final Intake intake;
  private final double speed;
  private final boolean brake;

  /** Creates a new MidDeploy. */
  public MidDeploy(Intake intake, Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    this.brake = (speed < 0);
    addRequirements(intake.arm);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.midDeploy(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopDeploy();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
