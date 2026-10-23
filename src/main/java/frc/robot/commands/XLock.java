// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Locks the swerve wheels in an X pattern (all modules pointed toward the robot's center) so the
 * robot resists being pushed. Bound with toggleOnTrue: one press locks, the next press unlocks, and the
 * default drive command resumes once it is unlocked. Can also be used in autos, e.g.
 * {@code new XLock(drivetrain).withTimeout(2.0)}.
 */
public class XLock extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;

  // Closed-loop velocity with a 0 target actively holds the drive wheels still.
  private final SwerveRequest.SwerveDriveBrake m_request =
      new SwerveRequest.SwerveDriveBrake().withDriveRequestType(DriveRequestType.Velocity);

  public XLock(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.setControl(m_request);
  }
  // Other drivetrain commands can't interrupt the lock; they are rejected instead.
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
  // No end() needed: the default drive command takes over when this command is interrupted.
}
