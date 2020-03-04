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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
 
  static XboxController m_button;
  private static CANSparkMax leftDriveMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax leftDriveSlave = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_TWO, MotorType.kBrushless);
  private static CANSparkMax rightDriveMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax rightDriveSlave = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_TWO, MotorType.kBrushless);
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  private static CANEncoder leftEncoder, rightEncoder;
  private static CANPIDController leftMotorPID, rightMotorPID;
  private static CANPIDController leftDrivePID, rightDrivePID;

  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  public static void setupMotors() {

    leftDrivePID = leftDriveMotor.getPIDController();
    rightDrivePID = rightDriveMotor.getPIDController(); 
    leftDriveMotor.setIdleMode(IdleMode.kCoast);
    leftDriveSlave.setIdleMode(IdleMode.kCoast);
    rightDriveMotor.setIdleMode(IdleMode.kCoast);
    rightDriveSlave.setIdleMode(IdleMode.kCoast);

    leftDriveSlave.follow(leftDriveMotor);
    rightDriveSlave.follow(rightDriveMotor);
    rightDriveSlave.setInverted(true);
    rightDriveMotor.setInverted(true);

    leftDriveMotor.setSmartCurrentLimit(80);
    leftDriveSlave.setSmartCurrentLimit(80);
    rightDriveMotor.setSmartCurrentLimit(80);
    rightDriveSlave.setSmartCurrentLimit(80);

    leftMotorPID = leftDriveMotor.getPIDController();
    rightMotorPID = rightDriveMotor.getPIDController();

    leftEncoder = leftDriveMotor.getEncoder();
    rightEncoder = rightDriveMotor.getEncoder();

    leftMotorPID.setP(Constants.LEFT_DRIVE_PID0_P, 0);
    leftMotorPID.setI(Constants.LEFT_DRIVE_PID0_I, 0);
    leftMotorPID.setD(Constants.LEFT_DRIVE_PID0_D, 0);
    leftMotorPID.setIZone(0, 0);
    leftMotorPID.setFF(Constants.LEFT_DRIVE_PID0_F, 0);
    leftMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    rightMotorPID.setP(Constants.RIGHT_DRIVE_PID0_P, 0);
    rightMotorPID.setI(Constants.RIGHT_DRIVE_PID0_I, 0);
    rightMotorPID.setD(Constants.RIGHT_DRIVE_PID0_D, 0);
    rightMotorPID.setIZone(0, 0);
    rightMotorPID.setFF(Constants.RIGHT_DRIVE_PID0_F, 0);
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
    return ((leftEncoder.getVelocity()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER) / Constants.SECONDS_PER_MINUTE;  }

  public static double getRightSpeed() {
    return ((rightEncoder.getVelocity()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER) / Constants.SECONDS_PER_MINUTE;
  }

  public static double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

//Get Distance in inches
public double getLeftDistance() {
    return (leftEncoder.getPosition()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER;
  }

  public static double getRightDistance(){
    return (rightEncoder.getPosition()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER;
  }

  public static double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public static double getRightPosition() {
    return rightEncoder.getPosition();
  }


  // Reset Encoders
  public void resetEncoders() {
    leftEncoder.setPosition(0); 
    rightEncoder.setPosition(0);
  }
    
  public double getHeading(){
    return gyro.getAngle();
  }

  public void setHeading(){
   gyro.reset();
  }

  public static void baselock() {

    double baselockLeft = getLeftPosition();
    double baselockRight = getRightPosition();

    leftMotorPID.setReference(baselockLeft, ControlType.kPosition);
    rightMotorPID.setReference(baselockRight, ControlType.kPosition);
  }

    public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Angle: " + getHeading());
  }
}
