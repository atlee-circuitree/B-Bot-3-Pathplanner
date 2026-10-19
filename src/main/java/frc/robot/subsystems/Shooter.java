// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.MotorLimits;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final TalonFX ShooterRight; // Follower
  private final TalonFX ShooterLeft;  // Leader

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withFeedForward(0).withSlot(0);

  // Filter setup
  private final LinearFilter m_velocityFilter = LinearFilter.movingAverage(5);
  private double m_filteredVelocity = 0;

  private double targetVelocity = 0;

  // We only need the signal from the Leader motor
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_ShooterLeftVelocity;

  /** Creates a new Shooter. */
  public Shooter() {
    ShooterRight = new TalonFX(Constants.CAN_IDS.ShooterRight, "B-bot");
    ShooterLeft = new TalonFX(Constants.CAN_IDS.ShooterLeft, "B-bot");
    MotorLimits.apply(ShooterLeft, Constants.CurrentLimits.SHOOTER_STATOR_AMPS, Constants.CurrentLimits.SHOOTER_SUPPLY_AMPS);
    MotorLimits.apply(ShooterRight, Constants.CurrentLimits.SHOOTER_STATOR_AMPS, Constants.CurrentLimits.SHOOTER_SUPPLY_AMPS);

    // SET UP FOLLOWER:
    // Right motor follows the Left motor.
    // The boolean parameter is whether it needs to spin the opposite direction.
    // (true = opposite direction, false = same direction)
    ShooterRight.setControl(new Follower(Constants.CAN_IDS.ShooterLeft, MotorAlignmentValue.Opposed));

    m_ShooterLeftVelocity = ShooterLeft.getVelocity();
  }

  public boolean isAtSpeed() {
    // Check our smoothed out, filtered velocity against the target
    return Math.abs(m_filteredVelocity - targetVelocity) < Constants.Shooter.SHOOTER_THRESHOLD_RPS;
  }

  public double getVelocity() {
    return m_filteredVelocity;
  }

  public void spin(double speed) {
    targetVelocity = speed;
    // Only send the velocity request to the Leader. The Follower copies automatically.
    ShooterLeft.setControl(m_velocityRequest.withVelocity(targetVelocity));
  }

  public void runAtPower(double power) {
    // Only apply power to the Leader
    ShooterLeft.set(power);
  }

  public void stopShooter() {
    // Stopping the Leader stops the Follower
    ShooterLeft.set(0);
  }

  @Override
  public void periodic() {
    // 1. Fetch the latest raw velocity from the Left motor
    BaseStatusSignal.refreshAll(m_ShooterLeftVelocity);
    double rawVelocity = m_ShooterLeftVelocity.getValueAsDouble();

    // 2. Pass it through the moving average filter to smooth it out
    m_filteredVelocity = m_velocityFilter.calculate(rawVelocity);
    
    // Optional: Print to SmartDashboard so you can tune your threshold
    SmartDashboard.putNumber("Filtered Shooter Velocity", m_filteredVelocity);
    SmartDashboard.putNumber("Target Velocity", targetVelocity);
  }
}