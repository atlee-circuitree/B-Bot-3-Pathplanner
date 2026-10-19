package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Sets the stator and supply current limits on a TalonFX (Kraken X60).
 *
 * <p>Only the current-limit settings are changed. Everything else on the motor (inversion, neutral
 * mode, PID gains saved from Tuner X) is left alone.
 *
 * <ul>
 *   <li><b>Stator limit</b>: caps the current inside the motor, which is torque. Limits wheel slip,
 *       heat, and stalls.
 *   <li><b>Supply limit</b>: caps the current pulled from the battery. This is the one that protects
 *       battery voltage and prevents brownouts.
 * </ul>
 *
 * <p>Limits belong to each motor, so a follower motor needs its own call.
 */
public final class MotorLimits {

  private MotorLimits() {}

  public static void apply(TalonFX motor, double statorAmps, double supplyAmps) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();

    // Read what is already on the motor first so we only change the four values below.
    motor.getConfigurator().refresh(limits);

    limits.StatorCurrentLimit = statorAmps;
    limits.StatorCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = supplyAmps;
    limits.SupplyCurrentLimitEnable = true;

    StatusCode status = motor.getConfigurator().apply(limits);
    if (!status.isOK()) {
      DriverStation.reportWarning(
          "Could not set current limits on TalonFX " + motor.getDeviceID() + ": " + status, false);
    }
  }
}
