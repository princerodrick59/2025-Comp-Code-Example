// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGroundIntakeConstants;

public class CoralGroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralGroundIntake. */

  private TalonFXS coralIntake;
  private TalonFX intakePivot;

  private TalonFXSConfiguration intakeConfigs;
  private TalonFXConfiguration pivotConfigs;

  private PositionVoltage positionRequest;

  // need to implemnt candi as the encoder

  private DigitalInput intakeBeamBreak;

  public CoralGroundIntakeSubsystem() {
    // Coral Ground Intake
    coralIntake = new TalonFXS(CoralGroundIntakeConstants.kCoralGroundIntakeID);
    // Coral Ground Intake Pivot
    intakePivot = new TalonFX(CoralGroundIntakeConstants.kCoralGroundPivotID);

    // Intake Beam Break
    intakeBeamBreak = new DigitalInput(CoralGroundIntakeConstants.kCoralGroundBeamBreakID);

    // Intake Configs
    intakeConfigs = new TalonFXSConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake));
    // Apply Intake Configs
    coralIntake.getConfigurator().apply(intakeConfigs);

    // Pivot Configs
    pivotConfigs = new TalonFXConfiguration()
                        .withSlot0(new Slot0Configs()
                                      .withKP(CoralGroundIntakeConstants.kCoralGroundPivotPIDValueP)
                                      .withKI(CoralGroundIntakeConstants.kCoralGroundPivotPIDValueI)
                                      .withKD(CoralGroundIntakeConstants.kCoralGroundPivotPIDValueD))
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.CounterClockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake));
    // Apply Pivot Configs
    intakePivot.getConfigurator().apply(pivotConfigs);

    // Position Request
    positionRequest = new PositionVoltage(0).withSlot(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  // Coral Ground Intake
  public void coralIntake() {
    coralIntake.set(CoralGroundIntakeConstants.kCoralGroundIntakeSpeed);
  }

  // Coral Ground Outtake
  public void coralOuttake() {
    coralIntake.set(-CoralGroundIntakeConstants.kCoralGroundIntakeSpeed);
  }

  // Coral Ground Intake Stop
  public void coralIntakeStop() {
    coralIntake.set(0);
  }

  // Intake Coral with Beam Break
  public void intakeCoralWithBeamBreak(){
    if(intakeBeamBreak.get()){
      coralIntake.set(CoralGroundIntakeConstants.kCoralGroundIntakeSpeed);
    }else{
      coralIntake.set(0);
    }
  }

  // Does the Coral Ground Intake have a Coral?
  @AutoLogOutput
  public boolean hasCoral(){
    return intakeBeamBreak.get();
  }

  // Pivot Intake Up
  public void pivotIntakeUp(){
    intakePivot.set(CoralGroundIntakeConstants.kCoralGroundPivotSpeed);
  }

  // Pivot Intake Down
  public void pivotIntakeDown(){
    intakePivot.set(-CoralGroundIntakeConstants.kCoralGroundPivotSpeed);
  }

  // Pivot Intake Stop
  public void pivotIntakeStop(){
    intakePivot.set(0);
  }

  // Pivot Intake Position
  public void pivotIntakePosition(double position){
    intakePivot.setControl(positionRequest.withPosition(position));
  }


  // Get Pivot Intake Position
  @AutoLogOutput
  public double getPivotIntakePosition(){
    return intakePivot.getPosition().getValueAsDouble();
  }

  // get pivot intake velocity
  @AutoLogOutput
  public double getPivotIntakeVelocity(){
    return intakePivot.getVelocity().getValueAsDouble();
  }

  // get pivot intake current
  @AutoLogOutput
  public double getPivotIntakeCurrent(){
    return intakePivot.getSupplyCurrent().getValueAsDouble();
  }

  // get intake Velocity
  @AutoLogOutput
  public double getIntakeVelocity(){
    return coralIntake.getVelocity().getValueAsDouble();
  }

  // get intake current
  @AutoLogOutput
  public double getIntakeCurrent(){
    return coralIntake.getSupplyCurrent().getValueAsDouble();
  }

  
  



  
}
