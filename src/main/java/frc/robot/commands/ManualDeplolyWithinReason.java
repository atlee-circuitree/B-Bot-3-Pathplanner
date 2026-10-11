// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualDeplolyWithinReason extends Command {
  private final Intake intake;
  private final double speed;
  private final boolean brake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualDeplolyWithinReason(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    this.brake = (speed < 0);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
 @Override
  public void initialize() {
    //if (brake){
   //   intake.deployBrake();
   // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.deployManualWithinReason(speed);
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

