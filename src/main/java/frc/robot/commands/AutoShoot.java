// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.KickUp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * Shoots from the current distance, with no aiming. Hold the button:
 * un-jam preamble, then the shooter spins to the speed for the distance to the HUB while the
 * kicker feeds. KickerUper already waits for the wheels to be at speed.
 *
 * Aiming is a separate button (AimAtHub). Hold both for aim + shoot.
 * Releasing the button cancels everything; each part stops its own motors.
 */
public final class AutoShoot {

  private static final double UNJAM_SHOOTER_POWER = -0.1;
  private static final double UNJAM_SECONDS = 0.25;
  private static final double SETTLE_SECONDS = 0.5;

  private AutoShoot() {}

  public static Command create(Vision vision, Shooter shooter, KickUp kickUp) {

    Command unjam =
        Commands.parallel(
                shooter.runEnd(() -> shooter.runAtPower(UNJAM_SHOOTER_POWER), shooter::stopShooter),
                kickUp.runEnd(kickUp::reverse, kickUp::stop))
            .withTimeout(UNJAM_SECONDS);

    return Commands.sequence(
        unjam,
        Commands.waitSeconds(SETTLE_SECONDS),
        Commands.parallel(new ShootFromDistance(shooter, vision), new KickerUper(kickUp, shooter)));
  }
}