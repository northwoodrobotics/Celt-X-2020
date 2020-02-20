/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;


public class ClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private static CANSparkMax leftClimbMotor = new CANSparkMax(Constants.LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
  private static CANSparkMax rightClimbMotor = new CANSparkMax(Constants.RIGHT_CLIMBER_MOTOR, MotorType.kBrushless);
  private static CANEncoder leftClimbEncoder, rightClimbEncoder;

  private static CANPIDController leftClimbPID, rightClimbPID;
  private static Solenoid climbCylinder;

  public static void setupMotors() {
    leftClimbMotor.setIdleMode(IdleMode.kCoast);
    rightClimbMotor.setIdleMode(IdleMode.kCoast);
    rightClimbMotor.setInverted(true);
    leftClimbMotor.setSmartCurrentLimit(50);
    rightClimbMotor.setSmartCurrentLimit(50);

    leftClimbEncoder = leftClimbMotor.getEncoder();
    rightClimbEncoder = rightClimbMotor.getEncoder();

    leftClimbPID = leftClimbMotor.getPIDController();
    rightClimbPID = rightClimbMotor.getPIDController();

    leftClimbPID.setP(Constants.LEFT_CLIMBER_PID0_P, 0);
    leftClimbPID.setI(Constants.LEFT_CLIMBER_PID0_I, 0);
    leftClimbPID.setD(Constants.LEFT_CLIMBER_PID0_D, 0);
    leftClimbPID.setIZone(0, 0);
    leftClimbPID.setFF(Constants.LEFT_CLIMBER_PID0_F, 0);
    leftClimbPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    rightClimbPID.setP(Constants.RIGHT_CLIMBER_PID0_P, 0);
    rightClimbPID.setI(Constants.RIGHT_CLIMBER_PID0_I, 0);
    rightClimbPID.setD(Constants.RIGHT_CLIMBER_PID0_D, 0);
    rightClimbPID.setIZone(0, 0);
    rightClimbPID.setFF(Constants.RIGHT_CLIMBER_PID0_F, 0);
    rightClimbPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);

    climbCylinder = new Solenoid(Constants.CLIMB_CYLINDER);

  }

  public static void setIntakePosition(boolean out){
    climbCylinder.set(out);
  }

  public static void climbExtend() {

    setIntakePosition(Constants.CLIMB_EXTEND);
  }

  public static void climbRetract() {

    setIntakePosition(Constants.CLIMB_RETRACT);
  }


  // Set Speed For Both
  public static void setSpeed(double left, double right) {
    leftClimbMotor.set(left);
    rightClimbMotor.set(right);
  }

  // Set Left & Right Speed
  public static void setLeftPosition(double speed) {
    double leftPosition = leftClimbEncoder.getPosition();
    leftPosition += 2*speed;
    System.out.println("left " + leftPosition);
    leftClimbPID.setReference(leftPosition, ControlType.kPosition);
  }

  public static void setRightPosition(double speed) {
    double rightPosition = rightClimbEncoder.getPosition();
    rightPosition += 2*speed;
    System.out.println("right " + rightPosition);

    leftClimbPID.setReference(rightPosition, ControlType.kPosition);
  }



    public ClimbSubsystem() {

  }

  @Override
  public void periodic() {
  }
}
