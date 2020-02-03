/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  static XboxController m_button;
  private static CANSparkMax leftDriveMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax leftDriveSlave = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_TWO, MotorType.kBrushless);
  private static CANSparkMax rightDriveMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax rightDriveSlave = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_TWO, MotorType.kBrushless);

  private CANEncoder leftEncoder, righEncoder;
  private CANPIDController leftMotorPID, rightMotorPID;
  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  public static void startMotors() {
    leftDriveSlave.follow(leftDriveMotor);
    rightDriveSlave.follow(rightDriveMotor);
    rightDriveSlave.setInverted(true);
    rightDriveMotor.setInverted(true);
    // leftDriveSlave1.setInverted(true);
    // leftDriveMotor.setInverted(true);
    leftDriveMotor.setSmartCurrentLimit(80);
    leftDriveSlave.setSmartCurrentLimit(80);
    rightDriveMotor.setSmartCurrentLimit(80);
    rightDriveSlave.setSmartCurrentLimit(80);
    m_button = new XboxController(0);
    leftDriveMotor.set(0.3);
    rightDriveMotor.set(0.3);
    leftDriveSlave.set(0.3);
    rightDriveSlave.set(0.3);
  }

  public static void stopMotors(){
    leftDriveMotor.set(0);
    rightDriveMotor.set(0);
    leftDriveSlave.set(0);
    rightDriveSlave.set(0);
  }

  public static void getLeftOutput(){
    leftDriveMotor.getOutputCurrent();
  }

  public static void getRightOutput(){
    rightDriveMotor.getOutputCurrent();
  }

public static void getLeftSpeed(){
  leftDriveMotor.get();
}

public static void getRightSpeed(){
  rightDriveMotor.get();
}

    public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
