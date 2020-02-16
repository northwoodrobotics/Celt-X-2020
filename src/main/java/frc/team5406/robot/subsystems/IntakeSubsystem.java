/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.team5406.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private static CANSparkMax intakeRollers = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
  private static CANSparkMax hopperBrushes = new CANSparkMax(Constants.HOPPER_BRUSH_MOTOR, MotorType.kBrushless);

  private static TalonSRX throatSerializerOne = new TalonSRX(Constants.THROAT_SERIALIZER_MOTOR_ONE);
  private static TalonSRX throatSerializerTwo = new TalonSRX(Constants.THROAT_SERIALIZER_MOTOR_TWO);

  private static Solenoid intakeCylinder;

  private static int intakePulseCount = 0;

  public static void setupMotors() {

    intakeRollers.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
    hopperBrushes.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);

    throatSerializerOne.enableCurrentLimit(true);
    throatSerializerOne.configContinuousCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    throatSerializerOne.configPeakCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    throatSerializerOne.configPeakCurrentDuration(Constants.PEAK_CURRENT_DURATION);

    throatSerializerTwo.enableCurrentLimit(true);
    throatSerializerTwo.configContinuousCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    throatSerializerTwo.configPeakCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    throatSerializerTwo.configPeakCurrentDuration(Constants.PEAK_CURRENT_DURATION);

    throatSerializerTwo.setInverted(false);
    throatSerializerOne.setInverted(true);

    intakeCylinder = new Solenoid(Constants.INTAKE_CYLINDER);
  }

  public static void setIntakeSpeed(double speed) {

    intakeRollers.set(speed);
  }

  public static void setBrushSpeed(double speed) {

    hopperBrushes.set(speed);
  }

  public static void setSerializerOutput(double output) {

    throatSerializerOne.set(ControlMode.PercentOutput, output);
    throatSerializerTwo.set(ControlMode.PercentOutput, output / 4);
  }

  public static void setSerializerCircle() {

    throatSerializerOne.set(ControlMode.PercentOutput, Constants.SERIALIZER_OUTPUT);
    throatSerializerTwo.set(ControlMode.PercentOutput, -1*Constants.SERIALIZER_OUTPUT / 4);
  }

  public static void stopRollers() {

    setIntakeSpeed(0);
  }

  public static void stopSerialize() {

    setSerializerOutput(0);
  }

  public static void stopIntake() {

    setIntakeSpeed(0);
    setBrushSpeed(0);
  }

  public static void setIntakePosition(boolean out){}

  public static void intakeExtend() {

    setIntakePosition(Constants.INTAKE_EXTEND);
    intakeCylinder.set(Constants.INTAKE_EXTEND);
  }

  public static void intakeRetract() {

    setIntakePosition(Constants.INTAKE_RETRACT);
    intakeCylinder.set(Constants.INTAKE_RETRACT);
  }

  public void spinBrushes() {

    setBrushSpeed(Constants.HOPPER_BRUSH_OUTPUT);
    //Water engineer and code mentor (Danny) says it's very important that spinIntake() be renamed gobbleBalls()
  } 

  public static void reverseIntake() {

    setIntakeSpeed(-1 * Constants.INTAKE_ROLLER_OUTPUT);
  }

  public static void serialize() {

    setSerializerOutput(Constants.SERIALIZER_OUTPUT);
  }

  public static void reverseSerialize() {

    setSerializerOutput(-1 * Constants.SERIALIZER_OUTPUT);
  }

  public static void spinRollers() {

    setIntakeSpeed(Constants.INTAKE_ROLLER_OUTPUT);
  }
  
  public static void pulseRollers(){
   
      intakePulseCount++;
    
      if(intakePulseCount >15 && intakePulseCount < 25){
        IntakeSubsystem.spinRollers();
      }else if(intakePulseCount > 25){
        intakePulseCount =0;
          IntakeSubsystem.stopIntake();
        }else{
          IntakeSubsystem.stopIntake();
        }
  }
  
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {

  }
}
