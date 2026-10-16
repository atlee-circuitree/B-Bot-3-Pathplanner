// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.OptionalDouble;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * Spins the shooter at a speed chosen from the distance to the HUB.
 *
 * <p>The speed comes from {@link Constants.Shooter#SHOT_TABLE}, interpolated between rows. If the
 * tag is briefly lost, the last speed is held. Before the first reading it uses
 * {@link Constants.Shooter#AUTO_FALLBACK_RPS}. The shooter stops when this command ends, matching
 * the existing {@code Shoot} command.
 *
 * <p>This command only spins the wheels; feeding is done by {@code KickerUper}, which already waits
 * for {@code shooter.isAtSpeed()}.
 */
public class ShootFromDistance extends Command {

  /** Distance (m) to speed (RPS). Returns the nearest row's speed outside the table's range. */
  private static final InterpolatingDoubleTreeMap SHOT_MAP = new InterpolatingDoubleTreeMap();

  static {
    for (double[] row : Constants.Shooter.SHOT_TABLE) {
      SHOT_MAP.put(row[0], row[1]);
    }
  }

  private final Shooter m_shooter;
  private final Vision m_vision;
  private double m_targetRps = Constants.Shooter.AUTO_FALLBACK_RPS;

  public ShootFromDistance(Shooter shooter, Vision vision) {
    m_shooter = shooter;
    m_vision = vision;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_targetRps = Constants.Shooter.AUTO_FALLBACK_RPS;
  }

  @Override
  public void execute() {
    OptionalDouble distance = m_vision.getDistanceMeters();
    if (distance.isPresent()) {
      m_targetRps = SHOT_MAP.get(distance.getAsDouble());
    }
    // else: keep the previous target

    m_shooter.spin(m_targetRps);
    SmartDashboard.putNumber("Shooter/Auto Target (RPS)", m_targetRps);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
