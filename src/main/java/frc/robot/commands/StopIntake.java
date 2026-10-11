// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake; 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopIntake extends Command {
  private Intake intake;
 /** Creates a new StopIntake. */
  public StopIntake(Intake StopIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = StopIntake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopWheels();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
