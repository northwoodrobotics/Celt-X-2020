/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.team5406.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private static CANSparkMax intakeRollers = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);

  private static CANSparkMax leftSerializer = new CANSparkMax(Constants.THROAT_SERIALIZER_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax rightSerializer = new CANSparkMax(Constants.THROAT_SERIALIZER_MOTOR_TWO, MotorType.kBrushless);
  private static CANSparkMax djSpinner = new CANSparkMax(Constants.DJ_SPINNER, MotorType.kBrushless);

  private static CANPIDController leftSerializerPID, rightSerializerPID, djSpinnerPID;
  private static CANEncoder leftSerializerEncoder, rightSerializerEncoder, djSpinnerEncoder; 

  private static Solenoid intakeCylinder, djSpinnerCylinder;

  private static int intakePulseCount = 0;

  public static void setupMotors() {

    leftSerializerEncoder = leftSerializer.getEncoder();
    rightSerializerEncoder = rightSerializer.getEncoder();
    djSpinnerEncoder = djSpinner.getEncoder();
    leftSerializerPID = leftSerializer.getPIDController();
    rightSerializerPID = rightSerializer.getPIDController();
    djSpinnerPID = djSpinner.getPIDController();
    djSpinner.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
    intakeRollers.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
    intakeRollers.setIdleMode(IdleMode.kBrake);

    leftSerializer.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);

    rightSerializer.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);

    djSpinnerPID.setP(Constants.DJ_SPINNER_PID0_P, 0);
    djSpinnerPID.setI(Constants.DJ_SPINNER_PID0_I, 0);
    djSpinnerPID.setD(Constants.DJ_SPINNER_PID0_D, 0);
    djSpinnerPID.setIZone(0, 0);
    djSpinnerPID.setFF(Constants.DJ_SPINNER_PID0_F, 0);
    djSpinnerPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.DJ_SPINNER_ROTATION_SMART_MOTION_SLOT);
    djSpinnerPID.setSmartMotionAllowedClosedLoopError(.5, Constants.DJ_SPINNER_ROTATION_SMART_MOTION_SLOT);

    djSpinnerPID.setSmartMotionMaxVelocity(1920, 0);
    djSpinnerPID.setSmartMotionMaxAccel(1000, 0);
    djSpinnerPID.setSmartMotionAllowedClosedLoopError(0.2, 0);


   leftSerializerPID.setP(Constants.LEFT_SERIALIZER_PID0_P);
   leftSerializerPID.setI(Constants.LEFT_SERIALIZER_PID0_I, 0);
   leftSerializerPID.setD(Constants.LEFT_SERIALIZER_PID0_D, 0);
   leftSerializerPID.setIZone(0, 0);
   leftSerializerPID.setFF(Constants.LEFT_SERIALIZER_PID0_F, 0);
   leftSerializerPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

  rightSerializerPID.setP(Constants.RIGHT_SERIALIZER_PID0_P);
  rightSerializerPID.setI(Constants.RIGHT_SERIALIZER_PID0_I, 0);
  rightSerializerPID.setD(Constants.RIGHT_SERIALIZER_PID0_D, 0);
  rightSerializerPID.setIZone(0, 0);
  rightSerializerPID.setFF(Constants.RIGHT_SERIALIZER_PID0_F, 0);
  rightSerializerPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);


    leftSerializer.setInverted(false);
    rightSerializer.setInverted(true);

    intakeCylinder = new Solenoid(Constants.INTAKE_CYLINDER);
    djSpinnerCylinder = new Solenoid(Constants.DJ_SPINNER_CYLINDER);
  }

  public static void setIntakeSpeed(double speed) {

    intakeRollers.set(speed);
  }


  public static void setSerializerOutput(double left, double right) {
    if (left ==0 || right == 0) { //fix
      stopSerialize();

  } else {
    leftSerializerPID.setReference(left *  Constants.LEFT_SERIALIZER_GEAR_RATIO, ControlType.kVelocity);
    rightSerializerPID.setReference(right *  Constants.RIGHT_SERIALIZER_GEAR_RATIO, ControlType.kVelocity);

  }
  }
  public static void setDJSpinnerOutput(double rpm) {
    if (rpm ==0 ) {
      stopDJSpinner();
  } else {
    djSpinnerPID.setReference(rpm *  Constants.DJ_SPINNER_GEAR_RATIO, ControlType.kSmartMotion, Constants.DJ_SPINNER_ROTATION_SMART_MOTION_SLOT);

  }
  

  }

  public static void spinDJSpinner() {
    setDJSpinnerOutput(Constants.DJ_SPINNER_SPEED * Constants.COLOUR_WHEEL_DIAMETER/Constants.DJ_SPINNER_DIAMETER);
  }

  public static void spinDjSpinnerRotation(){
    djSpinnerPID.setReference(Constants.DJ_SPINNER_ROTATION_CONTROL_RPM, ControlType.kSmartMotion, Constants.DJ_SPINNER_ROTATION_SMART_MOTION_SLOT);
  }

  public static void resetDJSpinnerEncoder(){
    djSpinnerEncoder.setPosition(0);
  }

  public static double getDJSpinnerPosition() {
    return djSpinnerEncoder.getPosition();
  }

  public static double getDJSpinnerVelocity(){
    return djSpinnerEncoder.getVelocity();
  }
  public static void setSerializerCircle() {

    //leftSerializer.set(Constants.SERIALIZER_OUTPUT);
    //rightSerializer.set(-1*Constants.SERIALIZER_OUTPUT / 2);
  }

  public static void stopRollers() {

    setIntakeSpeed(0);
  }

  public static void stopSerialize() {

    leftSerializer.set(0);
    rightSerializer.set(0);
  }

  public static void stopIntake() {

    setIntakeSpeed(0);
  }

  public static void stopDJSpinner() {

    djSpinner.set(0);
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

  public static void djSpinnerUp(){
    djSpinnerCylinder.set(Constants.DJ_SPINNER_UP);
  }

  public static void djSpinnerDown(){
    djSpinnerCylinder.set(Constants.DJ_SPINNER_DOWN);
  }

  public static void reverseIntake() {

    setIntakeSpeed(-1 * Constants.INTAKE_ROLLER_OUTPUT);
  }

  public static void serialize() {

    setSerializerOutput(Constants.LEFT_SERIALIZER_OUTPUT, Constants.RIGHT_SERIALIZER_OUTPUT);
  }

  public static void reverseSerialize() {
    setSerializerOutput(-1*Constants.LEFT_SERIALIZER_OUTPUT, -1*Constants.RIGHT_SERIALIZER_OUTPUT);
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
