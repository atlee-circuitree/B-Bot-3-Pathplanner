// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {

  private Intake intake;
  private String action;
  private double val;

  /** Creates a new DeployIntake. */
  public IntakeCommand(Intake intake, double val, String action) {
    this.intake = intake;
    this.val = val;
    this.action = action;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  // public void execute() {
  //   if (action == "deploy") {
  //     intake.deployToPosition(val);
  //   } else {
  //     intake.intake();
  //   } 
  // }
  public void execute() {
    // FIX 2: Use .equals() for string comparison in Java
    if (action.equals("deploy")) {
      intake.deployManualWithinReason(val);
    } else {
      intake.intake();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (action == "deploy") {
      intake.stopDeploy();
    } else {
      intake.stopWheels();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
