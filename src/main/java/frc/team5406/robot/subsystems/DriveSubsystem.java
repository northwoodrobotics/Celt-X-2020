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
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ControlType;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveSubsystem extends SubsystemBase {
 
  static XboxController m_button;
  private static CANSparkMax leftDriveMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax leftDriveSlave = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_TWO, MotorType.kBrushless);
  private static CANSparkMax rightDriveMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax rightDriveSlave = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_TWO, MotorType.kBrushless);
 static AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  private static CANEncoder leftEncoder, rightEncoder;
  private static CANPIDController leftMotorPID, rightMotorPID;
  private static DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

static  DifferentialDriveOdometry odometry;
static SimpleMotorFeedforward driveTrain = 
  new SimpleMotorFeedforward(Constants.S_VOLTS,
                             Constants.V_VOLTS,
                             Constants.A_VOLTS);
 static Pose2d pose = new Pose2d();

  public static void setupMotors() {
    leftDriveMotor.setIdleMode(IdleMode.kBrake);
    leftDriveSlave.setIdleMode(IdleMode.kBrake);
    rightDriveMotor.setIdleMode(IdleMode.kBrake);
    rightDriveSlave.setIdleMode(IdleMode.kBrake);
    leftDriveSlave.follow(leftDriveMotor);
    rightDriveSlave.follow(rightDriveMotor);
   rightDriveMotor.setInverted(false);
   leftDriveMotor.setInverted(true);
   drive.setSafetyEnabled(false);

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
   rightMotorPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
  setHeading();
  }

  public void arcadeDrive(double speed, double turn){
    drive.arcadeDrive(turn, -1*speed);
  }
/*
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
  }*/

  // Stop Motors
  public static void stopMotors() {
    leftDriveMotor.stopMotor();
    rightDriveMotor.stopMotor();
    leftDriveSlave.stopMotor();
    rightDriveSlave.stopMotor();
  }/*

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
*/

//Get Distance
public double getLeftDistance() {
    return Units.inchesToMeters((leftEncoder.getPosition()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER);
}
public double getRightDistance(){
  return Units.inchesToMeters((rightEncoder.getPosition()/Constants.DRIVE_GEAR_RATIO)*Math.PI*Constants.DRIVE_WHEEL_DIAMETER);
}

  // Reset Encoders
  public static void resetEncoders() {
    leftEncoder.setPosition(0); 
    rightEncoder.setPosition(0);
  }
    

public static Rotation2d getHeading() {
  return Rotation2d.fromDegrees(gyro.getAngle() * (Constants.GYRO_REVERSED ? -1.0 : 1.0));
}

public static void setHeading(){
   gyro.zeroYaw();
}

    public DriveSubsystem() {
      odometry = new DifferentialDriveOdometry(getHeading()); 
      setupMotors();
  }

  public void outputSpeeds(double leftSpeed, double rightSpeed) {
    double origLeftSpeed = leftSpeed;
    double origRightSpeed = rightSpeed;
   leftSpeed /= Units.inchesToMeters(Constants.INCHES_PER_REV / Constants.SECONDS_PER_MINUTE);
  rightSpeed /= Units.inchesToMeters(Constants.INCHES_PER_REV / Constants.SECONDS_PER_MINUTE);
    //System.out.println("Left Speed, " + leftSpeed);
    //System.out.println("Right Speed, "+ rightSpeed);
    SmartDashboard.putNumber("Orig Left Speed" , origLeftSpeed);
    SmartDashboard.putNumber("Orig Right Speed" , origRightSpeed);
    SmartDashboard.putNumber("Left Speed" , leftSpeed);
    SmartDashboard.putNumber("Right Speed" , rightSpeed);

    SmartDashboard.putNumber("X Translation" , pose.getTranslation().getX());
    SmartDashboard.putNumber("Left Speed (A)" , leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed (A)" , rightEncoder.getVelocity());
    
    double arbFFLeft = driveTrain.calculate(origLeftSpeed);
    double arbFFRight = driveTrain.calculate(origRightSpeed);
    SmartDashboard.putNumber("Arb FF L" ,arbFFLeft);
    SmartDashboard.putNumber("Arb FF R" , arbFFRight);


    leftMotorPID.setReference(leftSpeed, ControlType.kVelocity, 0, arbFFLeft, CANPIDController.ArbFFUnits.kVoltage);
    rightMotorPID.setReference(rightSpeed, ControlType.kVelocity, 0, arbFFRight, CANPIDController.ArbFFUnits.kVoltage); 
    //System.out.println("Pose: " + getPose());
    drive.feed();
  }
  
 /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDriveMotor.setVoltage(leftVolts);
    rightDriveMotor.setVoltage(rightVolts);
    drive.feed();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void reset() {
    resetEncoders();
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  public static void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Left Distance " + getLeftDistance());
    //System.out.println("Right Distance " + getRightDistance());

  pose = odometry.update(getHeading(), getLeftDistance(), getRightDistance());

  }
}
