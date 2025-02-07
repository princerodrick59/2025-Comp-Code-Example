// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */

  // End Effector Intake
  private TalonFXS endEffectorIntake;
  // End Effector Pivot
  private TalonFX endEffectorPivot;

  // End Effector Configs
  private TalonFXSConfiguration intakeConfigs;
  // Pivot Configs
  private TalonFXConfiguration pivotConfigs;
  
  // Intake Beam Break
  private DigitalInput intakeBeamBreak;
  // Pivot Encoder
  private DutyCycleEncoder pivotEncoder;
  // Need to implement candi as the encoder

  // Position Request
  private PositionVoltage positionRequest;

  public EndEffector() {
    // End Effector Intake
    endEffectorIntake = new TalonFXS(EndEffectorConstants.kEndEffectorIntakeID);
    // End Effector Pivot
    endEffectorPivot = new TalonFX(EndEffectorConstants.kEndEffectorPivotID);

    // Intake Configs
    intakeConfigs = new TalonFXSConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake));

    // Apply Intake Configs
    endEffectorIntake.getConfigurator().apply(intakeConfigs);

    // Pivot Configs
    pivotConfigs = new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                      .withKP(EndEffectorConstants.kIntakePivotPIDValueP)
                                      .withKI(EndEffectorConstants.kIntakePivotPIDValueI)
                                      .withKD(EndEffectorConstants.kIntakePivotPIDValueD))
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.CounterClockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake));
    // Apply Pivot Configs
    endEffectorPivot.getConfigurator().apply(pivotConfigs);
  
    // Pivot Position Request
    positionRequest = new PositionVoltage(0).withSlot(0);

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // End Effector Intake
  public void endEffectorIntake() {
    endEffectorIntake.set(EndEffectorConstants.kIntakeSpeed);
  }

  // End Effector Outake
  public void endEffectorOutake() {
    endEffectorIntake.set(-EndEffectorConstants.kIntakeSpeed);
  }

  // End Effector Stop
  public void endEffectorStop() {
    endEffectorIntake.set(0);
  }

  // Intake with current spike detection
  public void intakeCoral() {
    if (endEffectorIntake.getSupplyCurrent().getValueAsDouble() > EndEffectorConstants.kIntakeCurrentSpike) {
      endEffectorIntake.set(0);
    } else {
      endEffectorIntake.set(EndEffectorConstants.kIntakeSpeed);
    }
  }

  // Intake with beam break detection
  public void intakeAlgae() {
    if (intakeBeamBreak.get()) {
      endEffectorIntake.set(0);
    } else {
      endEffectorIntake.set(EndEffectorConstants.kIntakeSpeed);
    }
  }

  // Do we have coral?
  public boolean hasCoral(){
    return endEffectorIntake.getSupplyCurrent().getValueAsDouble() > EndEffectorConstants.kIntakeCurrentSpike;
  }

  // Do we have algae?
  public boolean hasAlgae(){
    return intakeBeamBreak.get();
  }


  // End Effector Pivot Up
  public void EndEffectorPivotUp() {
    endEffectorPivot.set(EndEffectorConstants.kPivotSpeed);
  }

  // End Effector Pivot Down
  public void EndEffectorPivotDown() {
    endEffectorPivot.set(-EndEffectorConstants.kPivotSpeed);
  }

  // End Effector Pivot Stop
  public void EndEffectorPivotStop() {
    endEffectorPivot.set(0);
  } 

  // End Effector Pivot Position Control
  public void setPivotPosition(double position) {
    endEffectorPivot.setControl(positionRequest.withPosition(position));
  }


  // Get pivot position
  public double getEndEffectorPivot() {
    return endEffectorPivot.get();
  }

  // Get end effector intake
  public double getEndEffectorIntake() {
    return endEffectorIntake.get();
  }
  




}
