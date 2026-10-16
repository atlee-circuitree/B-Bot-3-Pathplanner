// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Rotates the robot to center the HUB AprilTag in the Limelight view while the driver keeps full
 * control of translation.
 *
 * <p>Runs for as long as it is scheduled (bind it with {@code whileTrue}). If no HUB tag is
 * visible, rotation falls back to the driver's own rotation input so the driver can turn to find
 * the tag.
 *
 * <p>Sign convention: Limelight tx is positive when the target is to the right. Positive swerve
 * rotation is counterclockwise. A target on the right therefore needs a negative (clockwise) rate,
 * which is exactly what {@code pid.calculate(tx, 0)} returns.
 */
public class AimAtHub extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;
  private final Vision m_vision;
  private final DoubleSupplier m_velocityX;
  private final DoubleSupplier m_velocityY;
  private final DoubleSupplier m_manualRotationRate;

  private final PIDController m_aimPid = new PIDController(LimelightConstants.AIM_KP, 0.0, 0.0);

  /**
   * Own request object on purpose: the shared teleop request has a rotational deadband of about
   * 0.66 rad/s, which would swallow the small corrections needed to finish aiming. Translation
   * keeps the same deadband as the teleop request.
   */
  private final SwerveRequest.FieldCentric m_request =
      new SwerveRequest.FieldCentric()
          .withDeadband(
              (Constants.Drive.MAX_SPEED_MPS / Constants.Drive.SPEED_DIVISOR)
                  * Constants.Drive.DEADBAND_PERCENT)
          .withRotationalDeadband(0.0)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * @param velocityX driver field-relative X velocity in m/s (same scaling as the default drive command)
   * @param velocityY driver field-relative Y velocity in m/s
   * @param manualRotationRate driver rotation rate in rad/s, used only while no tag is visible
   */
  public AimAtHub(
      CommandSwerveDrivetrain drivetrain,
      Vision vision,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier manualRotationRate) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_velocityX = velocityX;
    m_velocityY = velocityY;
    m_manualRotationRate = manualRotationRate;
    // Only the drivetrain is required: this interrupts the default drive command while held.
    // Vision is read-only, so it is deliberately not a requirement.
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_aimPid.reset();
  }

  @Override
  public void execute() {
    double rotationRate;
    if (m_vision.hasTarget()) {
      rotationRate =
          MathUtil.clamp(
              m_aimPid.calculate(m_vision.getTxDegrees(), 0.0),
              -LimelightConstants.AIM_MAX_RATE_RAD_PER_SEC,
              LimelightConstants.AIM_MAX_RATE_RAD_PER_SEC);
    } else {
      // Do not keep steering on old data: hand rotation back to the driver.
      m_aimPid.reset();
      rotationRate = m_manualRotationRate.getAsDouble();
    }

    m_drivetrain.setControl(
        m_request
            .withVelocityX(m_velocityX.getAsDouble())
            .withVelocityY(m_velocityY.getAsDouble())
            .withRotationalRate(rotationRate));
  }

  // No end() needed: when this command stops, the drivetrain's default (teleop) command resumes.

  @Override
  public boolean isFinished() {
    return false;
  }
}
