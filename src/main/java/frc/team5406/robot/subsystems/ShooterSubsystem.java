/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import frc.team5406.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private static CANSparkMax shooterMaster = new CANSparkMax(Constants.SHOOTER_WHEEL_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax shooterSlave = new CANSparkMax(Constants.SHOOTER_WHEEL_MOTOR_TWO, MotorType.kBrushless);
  private static CANSparkMax booster = new CANSparkMax(Constants.BOOSTER_ROLLER_MOTOR, MotorType.kBrushless);
  private static CANSparkMax hood = new CANSparkMax(Constants.HOOD_MOTOR, MotorType.kBrushless);
  private static CANSparkMax turret = new CANSparkMax(Constants.TURRET_AZIMUTH_MOTOR, MotorType.kBrushless);
  
  private static TalonSRX feeder = new TalonSRX(Constants.FEEDER_MOTOR);

  private static CANCoder turretEncoder, hoodAbsoluteEncoder;
  private static CANEncoder shooterEncoder, boosterEncoder, hoodEncoder;
  private static CANPIDController shooterPID, boosterPID, hoodPID;
  
  public static void setupMotors() {

    shooterEncoder = shooterMaster.getEncoder();
    boosterEncoder = booster.getEncoder();
    hoodEncoder = hood.getEncoder();
    turretEncoder = new CANCoder(Constants.TURRET_ENCODER);
    hoodAbsoluteEncoder = new CANCoder(Constants.HOOD_ENCODER);

    shooterPID = shooterMaster.getPIDController();
    boosterPID = booster.getPIDController();
    hoodPID = booster.getPIDController();

    shooterSlave.follow(shooterMaster, true);
    feeder.setInverted(true);

    shooterPID.setP(Constants.SHOOTER_PID0_P, 0);
    shooterPID.setI(Constants.SHOOTER_PID0_I, 0);
    shooterPID.setD(Constants.SHOOTER_PID0_D, 0);
    shooterPID.setIZone(0, 0);
    shooterPID.setFF(Constants.SHOOTER_PID0_F, 0);
    shooterPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    boosterPID.setP(Constants.BOOSTER_PID0_P);
    boosterPID.setI(Constants.BOOSTER_PID0_I, 0);
    boosterPID.setD(Constants.BOOSTER_PID0_D, 0);
    boosterPID.setIZone(0, 0);
    boosterPID.setFF(Constants.BOOSTER_PID0_F, 0);
    boosterPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    hoodPID.setP(Constants.HOOD_PID0_P, 0);
    hoodPID.setI(Constants.HOOD_PID0_I);
    hoodPID.setD(Constants.HOOD_PID0_D, 0);
    hoodPID.setIZone(0, 0);
    hoodPID.setFF(Constants.HOOD_PID0_F, 0);
    hoodPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    shooterMaster.setClosedLoopRampRate(Constants.SHOOTER_CLOSED_LOOP_RAMP_RATE);
    booster.setClosedLoopRampRate(Constants.SHOOTER_CLOSED_LOOP_RAMP_RATE);
    hood.setClosedLoopRampRate(Constants.SHOOTER_CLOSED_LOOP_RAMP_RATE);
    
    shooterMaster.setSmartCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT);
    shooterSlave.setSmartCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT);
    booster.setSmartCurrentLimit(Constants.BOOSTER_CURRENT_LIMIT);
    hood.setSmartCurrentLimit(Constants.HOOD_CURRENT_LIMIT);

    feeder.enableCurrentLimit(true);
    feeder.configContinuousCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    feeder.configPeakCurrentLimit(Constants.BAG_CURRENT_LIMIT);
    feeder.configPeakCurrentDuration(Constants.PEAK_CURRENT_DURATION);

    resetEncoders();
  }

  public static void resetEncoders() {

    shooterEncoder.setPosition(0);
    boosterEncoder.setPosition(0);
    hoodEncoder.setPosition(0);
  }

  public static void spinShooter(double RPM) {

      if (RPM == 0) {
        stopShooter();

    } else {
      shooterPID.setReference(RPM * 1 / Constants.SHOOTER_GEAR_RATIO, ControlType.kVelocity);
    }

  }

  public static double getShooterSpeed() {

    return shooterEncoder.getVelocity() * 1 / Constants.SHOOTER_GEAR_RATIO;
  }

  public static void spinBooster(double RPM) {

    if (RPM == 0) {
      stopBooster();

  } else {
    boosterPID.setReference(RPM * 1 / Constants.BOOSTER_GEAR_RATIO, ControlType.kVelocity);
    }

  }

  public static double getBoosterSpeed() {

    return boosterEncoder.getVelocity() * 1 / Constants.BOOSTER_GEAR_RATIO;
  }

  public static void setHoodAngle(double angle) {

    hoodPID.setReference(angle, ControlType.kPosition);
  }

  public static double getHoodAngle() {

    return hoodEncoder.getPosition();
  }

  public static void releaseHood() {

    hood.set(0);
  }

  public static double getHoodVoltage() {

    return hood.getAppliedOutput();
  }

  public static void stopShooter() {

    shooterMaster.set(0);
  }

  public static void stopBooster() {

    booster.set(0);
  }

  public static void spinFeeder() {
   
    feeder.set(ControlMode.PercentOutput, 0.8);
  }

  public static void stopFeeder() {

    feeder.set(ControlMode.PercentOutput, 0);
  }

  public static void getRPM() {

    //stuff goes in here to prep for more complex feeder
  }

  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {

  }
}
