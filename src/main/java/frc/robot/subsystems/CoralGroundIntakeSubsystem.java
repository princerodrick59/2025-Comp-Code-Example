// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGroundIntakeConstants;

public class CoralGroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralGroundIntake. */

  // Coral Ground Intake
  private TalonFXS coralIntake;
  private TalonFX intakePivot;
  // Intake Configs
  private TalonFXSConfiguration intakeConfigs;
  private TalonFXConfiguration pivotConfigs;
  // Pivot Encoder
  private DutyCycleEncoder pivotEncoder;
  // Intake Beam Break
  private DigitalInput intakeBeamBreak;
  // Pivot PID Controller
  private PIDController pivotPIDController;

  public CoralGroundIntakeSubsystem() {
    // Coral Ground Intake
    coralIntake = new TalonFXS(CoralGroundIntakeConstants.kCoralGroundIntakeID);
    // Coral Ground Intake Pivot
    intakePivot = new TalonFX(CoralGroundIntakeConstants.kCoralGroundPivotID);

    // Intake Beam Break
    intakeBeamBreak = new DigitalInput(CoralGroundIntakeConstants.kCoralGroundBeamBreakPort);
    // Pivot Encoder
    pivotEncoder = new DutyCycleEncoder(CoralGroundIntakeConstants.kCoralGroundPivotEncoderPort);

    // Intake Configs
    intakeConfigs = new TalonFXSConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake));
    // Apply Intake Configs
    coralIntake.getConfigurator().apply(intakeConfigs);

    // Pivot Configs
    pivotConfigs = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.CounterClockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake));
    // Apply Pivot Configs
    intakePivot.getConfigurator().apply(pivotConfigs);

    // Pivot PID Controller
    pivotPIDController = new PIDController( CoralGroundIntakeConstants.kCoralGroundPivotPIDValueP, 
                                            CoralGroundIntakeConstants.kCoralGroundPivotPIDValueI,
                                            CoralGroundIntakeConstants.kCoralGroundPivotPIDValueD);
    
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
  @AutoLogOutput(key = "CoralGroundIntakeSubsystem/Intake/HasCoral?")
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
    intakePivot.set(pivotPIDController.calculate(pivotEncoder.get(), position));
  }


  // Get Pivot Intake Position
  @AutoLogOutput(key = "CoralGroundIntakeSubsystem/IntakePivot/Position")
  public double getPivotIntakePosition(){
    return intakePivot.getPosition().getValueAsDouble();
  }

  // get pivot intake velocity
  @AutoLogOutput(key = "CoralGroundIntakeSubsystem/IntakePivot/Velocity")
  public double getPivotIntakeVelocity(){
    return intakePivot.getVelocity().getValueAsDouble();
  }

  // get pivot intake current
  @AutoLogOutput(key = "CoralGroundIntakeSubsystem/IntakePivot/Current")
  public double getPivotIntakeCurrent(){
    return intakePivot.getSupplyCurrent().getValueAsDouble();
  }

  // get intake Velocity
  @AutoLogOutput(key = "CoralGroundIntakeSubsystem/Intake/Velocity")
  public double getIntakeVelocity(){
    return coralIntake.getVelocity().getValueAsDouble();
  }

  // get intake current
  @AutoLogOutput(key = "CoralGroundIntakeSubsystem/Intake/Current")
  public double getIntakeCurrent(){
    return coralIntake.getSupplyCurrent().getValueAsDouble();
  }

  

  
  



  
}
