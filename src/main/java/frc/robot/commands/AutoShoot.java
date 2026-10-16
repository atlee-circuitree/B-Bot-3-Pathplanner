// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KickUp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * One-button "aim, set speed from distance, and shoot".
 *
 * <p>Everything runs together while the button is held:
 * <ul>
 *   <li>{@link AimAtHub} turns the robot toward the HUB the whole time (driver still drives).
 *   <li>The same un-jam preamble as the manual shoot buttons runs first (backs the note off the
 *       wheels), while the robot is already turning.
 *   <li>{@link ShootFromDistance} then spins the wheels to the speed for the current distance.
 *   <li>{@link KickerUper} feeds only once the robot is aimed, and it already waits for the wheels
 *       to be at speed. If no HUB tag is ever seen, nothing is fed.
 * </ul>
 * Releasing the button cancels everything; each part stops its own motors.
 */
public final class AutoShoot {

  // Same values as the manual shoot bindings in RobotContainer.
  private static final double UNJAM_SHOOTER_POWER = -0.1;
  private static final double UNJAM_SECONDS = 0.25;
  private static final double SETTLE_SECONDS = 0.5;

  private AutoShoot() {}

  public static Command create(
      CommandSwerveDrivetrain drivetrain,
      Vision vision,
      Shooter shooter,
      KickUp kickUp,
      DoubleSupplier velocityX,
      DoubleSupplier velocityY,
      DoubleSupplier manualRotationRate) {

    Command aim = new AimAtHub(drivetrain, vision, velocityX, velocityY, manualRotationRate);

    // runEnd stops the motors if the button is released mid-unjam.
    Command unjam =
        Commands.parallel(
                shooter.runEnd(() -> shooter.runAtPower(UNJAM_SHOOTER_POWER), shooter::stopShooter),
                kickUp.runEnd(kickUp::reverse, kickUp::stop))
            .withTimeout(UNJAM_SECONDS);

    Command feedWhenAimed =
        Commands.sequence(Commands.waitUntil(vision::isAimed), new KickerUper(kickUp, shooter));

    Command shootSequence =
        Commands.sequence(
            unjam,
            Commands.waitSeconds(SETTLE_SECONDS),
            Commands.parallel(new ShootFromDistance(shooter, vision), feedWhenAimed));

    return Commands.parallel(aim, shootSequence);
  }
}
