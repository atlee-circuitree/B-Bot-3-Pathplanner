// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  //private final CANcoder PivotEncoder;

  /** Creates a new Intake. */

 private final VoltageOut m_voltageOut = new VoltageOut(0);

 private final CANcoder PivotEncoder;

 private final StatusSignal<edu.wpi.first.units.measure.Angle> m_PivotPosition;
 private PIDController pid = new PIDController(0.03, 0, 0);

  public Intake() {

    IntakeWheels   = new TalonFX(Constants.CAN_IDS.IntakeWheels, "B-bot");
    IntakeArm = new TalonFX(Constants.CAN_IDS.IntakeArm, "B-bot");
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
  
  public void deployManualWithinReason(double speed) {
  double currentAngle = getAngleEncoder();

  // Safety Check: If we are at/past the minimum limit and trying to move lower (negative speed)
  if (currentAngle <= Constants.Intake.DEPLOY_MIN && speed > 0) {
    stopDeploy();
  }
  // Safety Check: If we are at/past the maximum limit and trying to move higher (positive speed)
  else if (currentAngle >= Constants.Intake.PRETTYDEPLOY_MAX && speed < 0) {
    stopDeploy();
  } 
  else if (currentAngle >= Constants.Intake.PRETTYDEPLOY_MID && speed < 0) {
    stopDeploy();
  }
  else {
    // Safe to move
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
