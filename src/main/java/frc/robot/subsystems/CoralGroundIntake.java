// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGroundIntake extends SubsystemBase {
  /** Creates a new CoralGroundIntake. */

  private TalonFXS coralIntake;
  private TalonFX intakePivot;

  private TalonFXSConfiguration intakeConfigs;
  private TalonFXConfiguration pivotConfigs;

  private PositionVoltage positionRequest;

  // need to implemnt candi as the encoder

  public CoralGroundIntake() {

    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
