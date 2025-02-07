// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  private TalonFXS endEffectorIntake;
  private TalonFX endEffectorPivot;

  private TalonFXSConfiguration intakeConfigs;
  private TalonFXConfiguration pivotConfigs;

  private DutyCycleEncoder pivotEncoder;

  public EndEffector() {

    endEffectorIntake = new TalonFXS(EndEffectorConstants.kEndEffectorIntakeID);
    endEffectorPivot = new TalonFX(EndEffectorConstants.kEndEffectorPivotID);

    intakeConfigs = new TalonFXSConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake));

    endEffectorIntake.getConfigurator().apply(intakeConfigs);

    pivotConfigs = new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                      .withKP(0)
                                      .withKI(0)
                                      .withKD(0))
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.CounterClockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake));

    endEffectorPivot.getConfigurator().apply(pivotConfigs);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
