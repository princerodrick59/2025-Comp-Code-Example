// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX elevatorLeftLeaderMotor;
  private TalonFX elevatorRightFollowerMotor;

  private TalonFXConfiguration elevatorConfigs;

  private Follower elevatorFollower;

  private MotionMagicVoltage elevatorPositionRequest;

  private Distance lastDesiredPosition;

  public ElevatorSubsystem() {
    elevatorLeftLeaderMotor = new TalonFX(18);
    elevatorRightFollowerMotor = new TalonFX(19);

    elevatorFollower = new Follower(18, false);
    elevatorRightFollowerMotor.setControl(elevatorFollower);

    elevatorPositionRequest = new MotionMagicVoltage(0);

    lastDesiredPosition = Units.Inches.of(0);

    // elevatorConfigs
    elevatorConfigs = new TalonFXConfiguration()
                          .withSlot0(new Slot0Configs()
                                        .withKP(0)
                                        .withKI(0)
                                        .withKD(0)
                                        .withKS(10)
                                        .withKV(0)
                                        .withKA(0)
                                        .withKG(0)
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withGravityType(GravityTypeValue.Elevator_Static))
                          .withFeedback(new FeedbackConfigs()
                                            .withSensorToMechanismRatio(0))//Need to get sensor to mechanism ratio
                          .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.Clockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake))
                          .withMotionMagic(new MotionMagicConfigs()
                                              .withMotionMagicCruiseVelocity(0)
                                              .withMotionMagicAcceleration(0)
                                              .withMotionMagicExpo_kV(0)); 
    // Apply elevatorConfigs
    elevatorLeftLeaderMotor.getConfigurator().apply(elevatorConfigs);
    elevatorRightFollowerMotor.getConfigurator().apply(elevatorConfigs);

    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorUp() {
    elevatorLeftLeaderMotor.set(0.25);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }

  public void elevatorDown() {
    elevatorLeftLeaderMotor.set(-0.25);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }

  public void elevatorStop() {
    elevatorLeftLeaderMotor.set(0);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }

  @AutoLogOutput
  public Distance getElevatorPosition() {
    return Units.Inches.of(elevatorLeftLeaderMotor.getPosition().getValueAsDouble());
  }
  @AutoLogOutput
  public double getElevatorLeftMotorVelocity() {
    return elevatorLeftLeaderMotor.getVelocity().getValueAsDouble();
  }
  @AutoLogOutput
  public double getElevatorRightMotorVelocity() {
    return elevatorRightFollowerMotor.getVelocity().getValueAsDouble();
  }
  @AutoLogOutput
  public Distance getLastDesiredPosition() {
    return lastDesiredPosition;
  }
  public void setElevatorPosition(Distance height) {
    elevatorLeftLeaderMotor.setControl(elevatorPositionRequest.withPosition(height.in(Units.Inches)));
    elevatorRightFollowerMotor.setControl(elevatorFollower);
    lastDesiredPosition = height;

  }
  @AutoLogOutput
  public boolean isAtSetpoint() {
    return (getElevatorPosition().compareTo(getLastDesiredPosition().minus(Units.Inches.of(0.5))) > 0) &&
            getElevatorPosition().compareTo(getLastDesiredPosition().plus(Units.Inches.of(0.5))) < 0;
  }
}
