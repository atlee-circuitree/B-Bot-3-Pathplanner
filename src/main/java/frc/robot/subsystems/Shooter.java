// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

/* 
public class Shooter extends SubsystemBase {

  private final TalonFX ShooterRight; //shooterRight
  private final TalonFX ShooterLeft; //shooterLeft

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withFeedForward(0).withSlot(0);

 
  private final LinearFilter m_velocityFilter = LinearFilter.movingAverage(5);
  private double m_filteredVelocity = 0;

  private double targetVelocity = 0;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_ShooterLeftVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_ShooterRightVelocity;


  // Creates a new Shooter.
  public Shooter() {

    ShooterRight = new TalonFX(Constants.CAN_IDS.ShooterRight, "B-bot");
    ShooterLeft = new TalonFX(Constants.CAN_IDS.ShooterLeft, "B-bot");

    m_ShooterLeftVelocity  = ShooterLeft.getVelocity();
    m_ShooterRightVelocity = ShooterRight.getVelocity();

  }

  public boolean isAtSpeed() {
    
    //SmartDashboard.putNumber("voltage", ShooterLeft.getMotorVoltage().getValueAsDouble());
   // return ShooterLeft.getMotorVoltage().getValueAsDouble() >= 6;
    //This bellow was commented
   //return Math.abs(getVelocity() - targetVelocity) < Constants.Shooter.SHOOTER_THRESHOLD_RPS;

   //This is the test one i added for just left with no filter.
    // Since they share a shaft, we only need to check the primary motor's velocity
    return Math.abs(m_ShooterLeftVelocity.getValueAsDouble() - targetVelocity) < Constants.Shooter.SHOOTER_THRESHOLD_RPS;
  }
  public double getVelocity() {
    return m_filteredVelocity;
  }

  public void spin(double speed) {
    targetVelocity = speed;
    //ShooterLeft.setControl(m_velocityRequest.withVelocity(targetVelocity));
    ShooterRight.setControl(m_velocityRequest.withVelocity(targetVelocity));
    ShooterLeft.setControl(m_velocityRequest.withVelocity(targetVelocity));
  }

  public void runAtPower(double power) {
    ShooterRight.set(power);
    ShooterLeft.set(power);
  }



  public void stopShooter() {
    //ShooterLeft.set(0);
    ShooterRight.set(0);
    ShooterLeft.set(0);
  }
  

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_ShooterLeftVelocity,m_ShooterRightVelocity);
    double rawVelocity = (m_ShooterLeftVelocity.getValueAsDouble() + m_ShooterRightVelocity.getValueAsDouble()) / 2.0;
    m_filteredVelocity = m_velocityFilter.calculate(rawVelocity);
    // This method will be called once per scheduler run
  }
}
*/

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