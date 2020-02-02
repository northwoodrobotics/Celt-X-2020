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

import com.ctre.phoenix.motorcontrol.can.*;

import frc.team5406.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMaster = new CANSparkMax(Constants.SHOOTER_WHEEL_MOTOR_ONE, MotorType.kBrushless);
  private CANSparkMax shooterSlave = new CANSparkMax(Constants.SHOOTER_WHEEL_MOTOR_TWO, MotorType.kBrushless);
  private CANSparkMax booster = new CANSparkMax(Constants.BOOSTER_ROLLER_MOTOR, MotorType.kBrushless);
  private CANSparkMax hood = new CANSparkMax(Constants.HOOD_MOTOR, MotorType.kBrushless);
  private CANSparkMax turret = new CANSparkMax(Constants.TURRET_AZIMUTH_MOTOR, MotorType.kBrushless);
  
  private TalonSRX feeder = new TalonSRX(Constants.FEEDER_MOTOR);

  private CANEncoder shooterEncoder, boosterEncoder, hoodEncoder;
  private CANPIDController shooterPID, boosterPID, hoodPID;
  
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {

  }
}
