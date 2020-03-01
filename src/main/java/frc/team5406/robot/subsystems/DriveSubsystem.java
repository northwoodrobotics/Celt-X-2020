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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
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
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  private CANEncoder leftEncoder, rightEncoder;
  private CANPIDController leftMotorPID, rightMotorPID;
  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  public static void setupMotors() {
    leftDriveMotor.setIdleMode(IdleMode.kCoast);
    leftDriveSlave.setIdleMode(IdleMode.kCoast);
    rightDriveMotor.setIdleMode(IdleMode.kCoast);
    rightDriveSlave.setIdleMode(IdleMode.kCoast);
    leftDriveSlave.follow(leftDriveMotor);
    rightDriveSlave.follow(rightDriveMotor);
    rightDriveSlave.setInverted(true);
    rightDriveMotor.setInverted(true);
    // leftDriveSlave1.setInverted(true);
    // leftDriveMotor.setInverted(true);
    leftDriveMotor.setSmartCurrentLimit(60);
    leftDriveSlave.setSmartCurrentLimit(60);
    rightDriveMotor.setSmartCurrentLimit(60);
    rightDriveSlave.setSmartCurrentLimit(60);
    leftDriveMotor.setOpenLoopRampRate(0.1);
    leftDriveSlave.setOpenLoopRampRate(0.1);
    rightDriveMotor.setOpenLoopRampRate(0.1);
    rightDriveSlave.setOpenLoopRampRate(0.1);
    
    m_button = new XboxController(0);
  }

  public void arcadeDrive(double speed, double turn){
    drive.arcadeDrive(-1*turn, speed);
  }

  // Set Speed For Both
  public void setSpeed(double left, double right) {
    leftDriveMotor.set(left);
    rightDriveMotor.set(right);
  }

  // Set Left & Right Speed
  public void setLeftSpeed(double speed) {
    leftDriveMotor.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightDriveMotor.set(speed);
  }

  // Set Output for Both Motors
  public void setOutput(double left, double right) {
    leftDriveMotor.setVoltage(left);
    rightDriveMotor.setVoltage(right);
  }

  // Set Left and Right Motors Output
  public void setLeftOutput(double output) {
    leftDriveMotor.setVoltage(output);
  }

  public void setRightOutput(double output) {
    rightDriveMotor.setVoltage(output);
  }

  // Stop Motors
  public static void stopMotors() {
    leftDriveMotor.stopMotor();
    rightDriveMotor.stopMotor();
    leftDriveSlave.stopMotor();
    rightDriveSlave.stopMotor();
  }

  // Get Left & Right Output
  public static double getLeftOutput() {
    return leftDriveMotor.getOutputCurrent();
  }

  public static double getRightOutput() {
    return rightDriveMotor.getOutputCurrent();
  }

  // Get Left & Right Speed
  public static double getLeftSpeed() {
    return leftDriveMotor.get();
  }

  public static double getRightSpeed() {
    return rightDriveMotor.get();
  }

//Get Distance
public double getLeftDistnace() {
    return (leftEncoder.getPosition()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER;
}
public double getRightDistance(){
  return (rightEncoder.getPosition()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER;
}

  // Reset Encoders
  public void resetEnoders() {
    leftEncoder.setPosition(0); 
    rightEncoder.setPosition(0);
}

//
public double getHeading(){
  return gyro.getAngle();
}
public void setHeading(){
   gyro.reset();
}

    public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Angle: " + getHeading());
  }
}
