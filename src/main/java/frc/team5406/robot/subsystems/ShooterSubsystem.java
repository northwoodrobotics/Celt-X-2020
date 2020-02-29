/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import frc.team5406.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private static CANSparkMax shooterMaster = new CANSparkMax(Constants.SHOOTER_WHEEL_MOTOR_ONE, MotorType.kBrushless);
  private static CANSparkMax shooterSlave = new CANSparkMax(Constants.SHOOTER_WHEEL_MOTOR_TWO, MotorType.kBrushless);
  private static CANSparkMax booster = new CANSparkMax(Constants.BOOSTER_ROLLER_MOTOR, MotorType.kBrushless);
  private static CANSparkMax hood = new CANSparkMax(Constants.HOOD_MOTOR, MotorType.kBrushless);
  private static CANSparkMax turret = new CANSparkMax(Constants.TURRET_AZIMUTH_MOTOR, MotorType.kBrushless);
  
  private static CANSparkMax upperFeeder = new CANSparkMax(Constants.UPPER_FEEDER_MOTOR, MotorType.kBrushless);

  private static CANCoder turretEncoder, hoodAbsoluteEncoder;
  private static CANEncoder shooterEncoder, boosterEncoder, hoodEncoder, feederEncoder;
  private static CANPIDController shooterPID, boosterPID, hoodPID, feederPID; 

  public static boolean llHasValidTarget = false;
  public static double llSteer = 0.0;

  public static double llLastError = 0; 
  public static double llTotalError = 0;

  private static double hoodAngle = 0;
  private static double rpm = 0; 
  public static double shooterMultiplier = 1;
 
  static Compressor compressor = new Compressor();
  
  public static void setupMotors() {

    
    shooterEncoder = shooterMaster.getEncoder();
    boosterEncoder = booster.getEncoder();
    hoodEncoder = hood.getEncoder();
    feederEncoder = upperFeeder.getEncoder();
    turretEncoder = new CANCoder(Constants.TURRET_ENCODER);
    hoodAbsoluteEncoder = new CANCoder(Constants.HOOD_ENCODER);

    shooterPID = shooterMaster.getPIDController();
    boosterPID = booster.getPIDController();
    hoodPID = hood.getPIDController();
    feederPID = upperFeeder.getPIDController();

    shooterSlave.follow(shooterMaster, true);

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

    feederPID.setP(Constants.FEEDER_PID0_P);
    feederPID.setI(Constants.FEEDER_PID0_I, 0);
    feederPID.setD(Constants.FEEDER_PID0_D, 0);
    feederPID.setIZone(0, 0);
    feederPID.setFF(Constants.FEEDER_PID0_F, 0);
    feederPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);

    shooterMaster.setClosedLoopRampRate(Constants.SHOOTER_CLOSED_LOOP_RAMP_RATE);
    booster.setClosedLoopRampRate(Constants.SHOOTER_CLOSED_LOOP_RAMP_RATE);
    hood.setClosedLoopRampRate(Constants.SHOOTER_CLOSED_LOOP_RAMP_RATE);
    
    shooterMaster.setSmartCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT);
    shooterSlave.setSmartCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT);
    booster.setSmartCurrentLimit(Constants.BOOSTER_CURRENT_LIMIT);
    hood.setSmartCurrentLimit(Constants.HOOD_CURRENT_LIMIT);

    upperFeeder.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);

    resetEncoders();
  }

  public static void resetEncoders() {

    shooterEncoder.setPosition(0);
    boosterEncoder.setPosition(0);
    hoodEncoder.setPosition(getAbsHoodPosition()*Constants.HOOD_GEAR_RATIO);
  }

  public static void spinShooter(double RPM) {
double f = SmartDashboard.getNumber("SHOOTER_PID_F", Constants.SHOOTER_PID0_F);
double p = SmartDashboard.getNumber("SHOOTER_PID_P", Constants.SHOOTER_PID0_P);
double d = SmartDashboard.getNumber("SHOOTER_PID_D", Constants.SHOOTER_PID0_D);
double i = SmartDashboard.getNumber("SHOOTER_PID_I", Constants.SHOOTER_PID0_I);

shooterPID.setFF(f);
shooterPID.setP(p);
shooterPID.setD(d);
shooterPID.setI(i);
/*System.out.println(f);
System.out.println(i);
System.out.println(d);
System.out.println(p);*/

      if (RPM == 0) {
        stopShooter();

    } else {
      shooterPID.setReference(RPM *  Constants.SHOOTER_GEAR_RATIO, ControlType.kVelocity);
    }

  }
  public static void spinShooterAuto(){
      spinShooter(rpm);
  }
  public static void setHoodAngleAuto(){
      setHoodAngle(hoodAngle);
  }

  public static double getShooterSpeed() {

    return shooterEncoder.getVelocity() * 1 / Constants.SHOOTER_GEAR_RATIO;
  }

  public static void changeShooterMultiplier(boolean increase) {
    shooterMultiplier*= 1+ (increase?1:-1) * Constants.SHOOTER_ADJUSTMENT;
  }

  public static void spinBooster(double RPM) {

    if (RPM == 0) {
      stopBooster();

  } else {
    boosterPID.setReference(RPM *  Constants.BOOSTER_GEAR_RATIO, ControlType.kVelocity);
    }

  }
  public static boolean checkRPM(){
    double realRPM = getShooterSpeed();
    if(realRPM * 0.95 < rpm && realRPM * 1.05 > rpm){
      return true;
    }
    else{
      return false;
    }
  }

  public static double getBoosterSpeed() {

    return boosterEncoder.getVelocity() * 1 / Constants.BOOSTER_GEAR_RATIO;
  }

  public static void setHoodAngle(double angle) {

    hoodPID.setReference(angle*Constants.HOOD_GEAR_RATIO, ControlType.kPosition);
  }

  public static double getHoodAngle() {

    return hoodEncoder.getPosition()/Constants.HOOD_GEAR_RATIO;
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

  public static void spinFeeder(double RPM) {
      if (RPM == 0) {
        stopShooter();

    } else {
      feederPID.setReference(RPM *  Constants.FEEDER_GEAR_RATIO, ControlType.kVelocity);
    }
    //upperFeeder.set(1);

  }

  public static double getFeederSpeed(){
    return feederEncoder.getVelocity() * 1 / Constants.FEEDER_GEAR_RATIO;
  }
  public static void reverseFeeder() {
   
    upperFeeder.set(-1 * Constants.UPPER_FEEDER_OUTPUT);

  }

  public static void stopFeeder() {

    upperFeeder.set(0);
  }

  public static void getRPM() {

    //stuff goes in here to prep for more complex feeder
  }

  public static void turnTurret(double turn){
    turret.set(turn);
  }

  public static double getAbsHoodPosition(){
    double absPos = hoodAbsoluteEncoder.getAbsolutePosition();
    if(absPos > 270){
      absPos -=360;
    }
    return absPos/Constants.HOOD_ENC_GEAR_RATIO;
  }

  public static void updateLimelightTracking()
{
      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
      SmartDashboard.putNumber("TY", ty);
      

    // ts: close to 0 - left, close to 90 - right
    // tx: negative - left, positive - right

    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_KP = 0.025;                    // how hard to turn toward the target
    final double STEER_KD = 0.005;
    final double STEER_KI = 0.1;
    final double MAX_DRIVE = 0.3;                   // Simple speed limit so we don't drive too fast

      if (tv < 1.0)
      {
        llHasValidTarget = false;
        llSteer = 0.0;
        rpm = 1500;
        hoodAngle = 0; 
        return;
      }
      double d = (67.5 / Math.tan(Units.degreesToRadians(30+ty)));
       hoodAngle = -0.0003*d*d + 0.185*d + 27.203;
       rpm = (-0.0271*d*d + 16.63*d + 2385.8) * shooterMultiplier;
       
       if(hoodAngle > 65){
         hoodAngle = 65;
       }
       if(hoodAngle < 0){
         hoodAngle = 0;
       }
       if(rpm > 6500){
        rpm = 6500;
       }
       if(rpm < 0){
         rpm = 0;
       }
       if(ts< -45){
         ts+= 90;
       }
       double txOff = ts/12;
       /*System.out.println("hood " + hoodAngle);
       System.out.println("rpm " + rpm);
       System.out.println("ty " + ty);
       System.out.println("d " + d);*/
      llHasValidTarget = true;
      llTotalError += tx-txOff;
      // llTotalError += tx; 
      // Start with proportional steering
      llSteer = (tx-txOff)* STEER_KP; //+ STEER_KD * (tx - llLastError) / 0.02 + STEER_KI * llTotalError * 0.02;
      //llSteer = tx * STEER_KP;

      // try to drive forward until the target area reaches our desired area
      
      llLastError = tx-txOff;
      //  llLastError = tx;
      if (Math.abs(llSteer) > MAX_DRIVE)
      {
        llSteer = Math.signum(llSteer) * MAX_DRIVE;
      }
}

  public double maxllArea(double angle){
return -0.0054*angle*angle + 0.6546*angle - 12.084;
}

public static void compressorEnabled() {
  compressor.start();
}
public static void compressorDisabled() {
  compressor.stop();
}

  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {

  }
}
