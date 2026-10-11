// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class KickUp extends SubsystemBase {
  
  public final TalonFX KickUpMotor;
  public final TalonFX Indexer;
  
   
  /** Creates a new KickUp. */
  public KickUp() {
    KickUpMotor = new TalonFX(Constants.CAN_IDS.KickUpMotor, "B-bot");
    Indexer = new TalonFX(Constants.CAN_IDS.Indexer, "B-bot");
  }

  public void shoot(){
    KickUpMotor.set(Constants.KickUp.KickUpSpeed);
    Indexer.set(Constants.KickUp.KickUpSpeed);
    
  }

  // This method is to run the motors backward
  public void reverse(){
    KickUpMotor.set(-0.2); 
    Indexer.set(-0.2);
  }

  public void stop(){
    KickUpMotor.set(0);
    Indexer.set(0);
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
