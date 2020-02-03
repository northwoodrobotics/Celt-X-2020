/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;


public class IntakeSubsystem extends SubsystemBase {
  
  private static CANSparkMax intakeRollers = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
  private static CANSparkMax hopperBrushes = new CANSparkMax(Constants.HOPPER_BRUSH_MOTOR, MotorType.kBrushless);
  private static TalonSRX throatSerializer = new TalonSRX(Constants.THROAT_SERIALIZER_MOTOR);


  public IntakeSubsystem() {
    setupMotors();
  }

  private void setupMotors(){
    intakeRollers.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
    hopperBrushes.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
    throatSerializer.enableCurrentLimit(true);
    throatSerializer.configContinuousCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    throatSerializer.configPeakCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    throatSerializer.configPeakCurrentDuration(Constants.PEAK_CURRENT_DURATION);

  }

  private void setIntakeRollerOutput(double output){
    intakeRollers.set(output);
  }

  private void setHopperBrushOutput(double output){
    hopperBrushes.set(output);
  }

  private void setSerializerOutput(double output){
    throatSerializer.set(ControlMode.PercentOutput, output);
  }

  private void setIntakePosition(boolean out){

  }

  public void retractIntake(){
    setIntakePosition(Constants.INTAKE_RETRACT);
  }

  public void extendIntake(){
    setIntakePosition(Constants.INTAKE_EXTEND);
  }

  public void intake(){
    setIntakeRollerOutput(Constants.INTAKE_ROLLER_OUTPUT);
    setHopperBrushOutput(Constants.HOPPER_BRUSH_OUTPUT);
  }

  public void serialize(){
    setSerializerOutput(Constants.HOPPER_BRUSH_OUTPUT);
  }

  public void outtake(){
    setIntakeRollerOutput(-1*Constants.INTAKE_ROLLER_OUTPUT);
  }

  public void reverseSerializer(){
    setSerializerOutput(-1*Constants.HOPPER_BRUSH_OUTPUT);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
