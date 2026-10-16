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
 * control of translation. Bind with whileTrue. If no HUB tag is visible, rotation falls back to
 * the driver's own rotation input.
 *
 * Sign convention: Limelight tx is positive when the target is to the right. Positive swerve
 * rotation is counterclockwise, so a target on the right needs a negative (clockwise) rate.
 */
public class AimAtHub extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;
  private final Vision m_vision;
  private final DoubleSupplier m_velocityX;
  private final DoubleSupplier m_velocityY;
  private final DoubleSupplier m_manualRotationRate;

  private final PIDController m_aimPid = new PIDController(LimelightConstants.AIM_KP, 0.0, 0.0);

  private final SwerveRequest.FieldCentric m_request =
      new SwerveRequest.FieldCentric()
          .withDeadband(
              (Constants.Drive.MAX_SPEED_MPS / Constants.Drive.SPEED_DIVISOR)
                  * Constants.Drive.DEADBAND_PERCENT)
          .withRotationalDeadband(0.0)
          .withDriveRequestType(DriveRequestType.Velocity);

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
      double txDeg = m_vision.getTxDegrees();
      double output = m_aimPid.calculate(txDeg, 0.0);
      // Small errors give tiny commands the wheels cannot act on, so add a minimum push
      // until the error is inside half the aimed tolerance.
      if (Math.abs(txDeg) > LimelightConstants.AIM_TOLERANCE_DEG / 2.0) {
        output += Math.copySign(LimelightConstants.AIM_MIN_RATE_RAD_PER_SEC, output);
      }
      rotationRate =
          MathUtil.clamp(
              output,
              -LimelightConstants.AIM_MAX_RATE_RAD_PER_SEC,
              LimelightConstants.AIM_MAX_RATE_RAD_PER_SEC);
    } else {
      m_aimPid.reset();
      rotationRate = m_manualRotationRate.getAsDouble();
    }

    m_drivetrain.setControl(
        m_request
            .withVelocityX(m_velocityX.getAsDouble())
            .withVelocityY(m_velocityY.getAsDouble())
            .withRotationalRate(rotationRate));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}