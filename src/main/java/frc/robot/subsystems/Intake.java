// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.MotorLimits;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {

  private final TalonFX IntakeWheels;
  private final TalonFX IntakeArm;
    /** Separate locks so a wheel command and an arm command can run at the same time. */
  public final Subsystem wheels = new SubsystemBase("IntakeWheels") {};
  public final Subsystem arm = new SubsystemBase("IntakeArm") {};
  //private final CANcoder PivotEncoder;

  /** Creates a new Intake. */

 private final VoltageOut m_voltageOut = new VoltageOut(0);

 private final CANcoder PivotEncoder;

 private final StatusSignal<edu.wpi.first.units.measure.Angle> m_PivotPosition;
 private PIDController pid = new PIDController(0.03, 0, 0);

  public Intake() {

    IntakeWheels   = new TalonFX(Constants.CAN_IDS.IntakeWheels, "B-bot");
    IntakeArm = new TalonFX(Constants.CAN_IDS.IntakeArm, "B-bot");
    MotorLimits.apply(IntakeWheels, Constants.CurrentLimits.INTAKE_WHEELS_STATOR_AMPS, Constants.CurrentLimits.INTAKE_WHEELS_SUPPLY_AMPS);
    MotorLimits.apply(IntakeArm, Constants.CurrentLimits.INTAKE_ARM_STATOR_AMPS, Constants.CurrentLimits.INTAKE_ARM_SUPPLY_AMPS);
    
    PivotEncoder = new CANcoder(Constants.CAN_IDS.PivotEncoder, "B-bot");

    Slot0Configs slot0ConfigsUp = new Slot0Configs();
    slot0ConfigsUp.kP = Constants.Intake.DEPLOY_SLOT0_KP;
    slot0ConfigsUp.kI = 0.0;
    slot0ConfigsUp.kD = 0.0;

    Slot1Configs slot1ConfigsDown = new Slot1Configs();
    slot1ConfigsDown.kP = Constants.Intake.DEPLOY_SLOT1_KP;
    slot1ConfigsDown.kI = 0.0;
    slot1ConfigsDown.kD = 0.0;

    m_PivotPosition = IntakeArm.getPosition();


  }

public void intake() {
    IntakeWheels.set(Constants.Intake.INTAKE_SPEED);
  }

public void outtake() {
    IntakeWheels.set(-Constants.Intake.INTAKE_SPEED);
  }

public void stopWheels() {
    IntakeWheels.setControl(m_voltageOut.withOutput(0));
  
  }
  
   public void deployManual(double speed) {
    IntakeArm.set(speed);
  }
    /** Which way a speed moves the "Pivot Encoder get" angle: +1 up, -1 down, 0 not moving. */
  private double angleDirection(double speed) {
    return Math.signum(speed) * Constants.Intake.ARM_SPEED_TO_ANGLE_SIGN;
  }

  public void deployManualWithinReason(double speed) {
    double angle = getAngleEncoder();
    double dir = angleDirection(speed);

    // Only block the direction that goes FURTHER past a limit; moving back is always allowed.
    boolean pastMin = angle <= Constants.Intake.DEPLOY_MIN && dir < 0;
    boolean pastMax = angle >= Constants.Intake.PRETTYDEPLOY_MAX && dir > 0;

    SmartDashboard.putBoolean("Intake Arm Blocked", pastMin || pastMax);
    if (pastMin || pastMax) {
      stopDeploy();
    } else {
      IntakeArm.set(speed);
    }
  }

  public void midDeploy(double speed) {
    double angle = getAngleEncoder();
    boolean pastMid = angle >= Constants.Intake.PRETTYDEPLOY_MID && angleDirection(speed) > 0;

    SmartDashboard.putBoolean("Intake Arm Blocked", pastMid);
    if (pastMid) {
      stopDeploy();
    } else {
      IntakeArm.set(speed);
    }
  }

  public void stopDeploy() {
    IntakeArm.set(0);
  }

  public void deployBrake() {
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    IntakeArm.getConfigurator().apply(motorOutput, 0.0);
  }

 
  //public double getAngle() {
   // return PivotEncoder.getPosition().getValueAsDouble();
 // }

  /*
  public void deployToPosition(double target) {
    if (target < Constants.Intake.DEPLOY_MIN && target < getAngle()) {
      stopDeploy();
    }
    if (target > Constants.Intake.DEPLOY_MAX && target > getAngle()) {
      stopDeploy();
    } else {
      if (getAngle() > (target + 2)) {
        IntakeArm.set((-1) * .25);
      } else if (getAngle() < (target - 2)) {
        IntakeArm.set((1) * .35);
      }
    }
  }
    */
public double getAngleEncoder() {
    return ((PivotEncoder.getPosition().getValueAsDouble() - Constants.Intake.DEPLOY_MAX) * 360 + 86.1) % 360;
    
    
    }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("deploy encoder read raw", PivotEncoder.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("deploy encoder read pretty",
    SmartDashboard.putNumber("Pivot Encoder get", getAngleEncoder());

  }

  
}
