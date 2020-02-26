/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team5406.robot.subsystems.DriveSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  XboxController operatorGamepad = new XboxController(Constants.OPERATOR_CONTROLLER);
  XboxController driverGamepad = new XboxController(Constants.DRIVER_CONTROLLER);
  private DriveSubsystem robotDrive = new DriveSubsystem();
  int intakePulseCount = 0;
 /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DriveSubsystem.setupMotors();
    ShooterSubsystem.setupMotors();
    IntakeSubsystem.setupMotors();
    ClimbSubsystem.setupMotors();
    SmartDashboard.putNumber("Shooter Target RPM", 5000);
    SmartDashboard.putNumber("Booster Target RPM", 6500);
    SmartDashboard.putNumber("Feeder Target RPM", 500);
    SmartDashboard.putNumber("Hood Target Angle", 0);
    
    SmartDashboard.putNumber("SHOOTER_PID_P", Constants.SHOOTER_PID0_P);
    SmartDashboard.putNumber("SHOOTER_PID_I", Constants.SHOOTER_PID0_I);
    SmartDashboard.putNumber("SHOOTER_PID_D", Constants.SHOOTER_PID0_D);
    SmartDashboard.putNumber("SHOOTER_PID_F", Constants.SHOOTER_PID0_F);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Hood Abs Position", ShooterSubsystem.getAbsHoodPosition());
    SmartDashboard.putNumber("Hood Angle", ShooterSubsystem.getHoodAngle());

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
   /* m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }*/
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    ShooterSubsystem.resetEncoders();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
 
    SmartDashboard.putNumber("ts", ts); 
    SmartDashboard.putNumber("tx", tx);
    robotDrive.arcadeDrive(operatorGamepad.getY(Hand.kLeft), operatorGamepad.getX(Hand.kRight));
    if(operatorGamepad.getBumperPressed(Hand.kRight) && operatorGamepad.getStartButtonPressed()){
      ClimbSubsystem.climbExtend();
    }
    if(operatorGamepad.getBumperPressed(Hand.kRight) && operatorGamepad.getBackButtonPressed()){
      ClimbSubsystem.climbRetract();
    }
    if (operatorGamepad.getBumperPressed(Hand.kRight) && (operatorGamepad.getY(Hand.kLeft) > 0.05)) { 
      double leftSpeed = operatorGamepad.getY(Hand.kLeft);
      double speedMultiplier = driverGamepad.getX(Hand.kLeft);
     /* System.out.println(leftSpeed);
      System.out.println(speedMultiplier);*/
      ClimbSubsystem.setSpeed(leftSpeed,leftSpeed*speedMultiplier);

      
    //ClimbSubsystem.setLeftPosition(leftSpeed);
    //ClimbSubsystem.setRightPosition(leftSpeed*speedMultiplier);
    }else{
      ClimbSubsystem.setSpeed(0,0);
    }

    SmartDashboard.putNumber("Shooter RPM", ShooterSubsystem.getShooterSpeed());
    SmartDashboard.putNumber("Feeder RPM", ShooterSubsystem.getBoosterSpeed());
    SmartDashboard.putNumber("Hood Output", ShooterSubsystem.getHoodVoltage());
    SmartDashboard.putNumber("Feeder RPM", ShooterSubsystem.getFeederSpeed());
    SmartDashboard.putNumber("Abs Hood", ShooterSubsystem.getAbsHoodPosition());

   if (operatorGamepad.getBButton() && !operatorGamepad.getBumperPressed(Hand.kRight)) { 
     // ShooterSubsystem.spinShooter(SmartDashboard.getNumber("Shooter Target RPM", 5000));
      ShooterSubsystem.spinBooster(SmartDashboard.getNumber("Booster Target RPM", 6500));
     // ShooterSubsystem.setHoodAngle(SmartDashboard.getNumber("Hood Target Angle", 0));
      ShooterSubsystem.compressorDisabled();
      ShooterSubsystem.spinShooterAuto();
      ShooterSubsystem.setHoodAngleAuto();

    } else if (operatorGamepad.getBButton() && operatorGamepad.getBumperPressed(Hand.kRight)) { 
      ShooterSubsystem.spinShooter(1200);
      ShooterSubsystem.spinBooster(6500);
      ShooterSubsystem.setHoodAngle(40);
      ShooterSubsystem.compressorDisabled();
    
     } else {
      ShooterSubsystem.compressorEnabled();
      ShooterSubsystem.stopShooter();
      ShooterSubsystem.stopBooster();
      ShooterSubsystem.releaseHood();
     }

     if (driverGamepad.getTriggerAxis(Hand.kRight) > .1){
      IntakeSubsystem.intakeExtend();
      IntakeSubsystem.spinRollers(); 
      //IntakeSubsystem.setSerializerCircle();
     }else if(!(operatorGamepad.getTriggerAxis(Hand.kRight) > .1)){
      IntakeSubsystem.intakeRetract();
      IntakeSubsystem.stopRollers();
     }
    
     if (operatorGamepad.getTriggerAxis(Hand.kRight) > .1) { 
       IntakeSubsystem.pulseRollers();
      ShooterSubsystem.spinFeeder(SmartDashboard.getNumber("Feeder Target RPM", 1000));
      //ShooterSubsystem.compressorDisabled(); 
      if(!(driverGamepad.getTriggerAxis(Hand.kRight) > .1)){
      IntakeSubsystem.serialize();
      }

     } else if (!(driverGamepad.getTriggerAxis(Hand.kLeft) > .1 )) {
      ShooterSubsystem.stopFeeder();
      //ShooterSubsystem.compressorEnabled();
      if(! (driverGamepad.getTriggerAxis(Hand.kRight) > .1)){
        IntakeSubsystem.stopSerialize();
      }
      
     }


     if(driverGamepad.getTriggerAxis(Hand.kLeft) > .1){
       IntakeSubsystem.reverseIntake();
       IntakeSubsystem.reverseSerialize();
       ShooterSubsystem.reverseFeeder();
     }

     if((Math.abs(operatorGamepad.getX(Hand.kRight)) > 0.1) && (operatorGamepad.getBumperPressed(Hand.kRight))){
      ShooterSubsystem.turnTurret(0.5 * operatorGamepad.getX(Hand.kRight));
     }
     else if(operatorGamepad.getAButton()){
      ShooterSubsystem.updateLimelightTracking();
     // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      if (ShooterSubsystem.llHasValidTarget){
        ShooterSubsystem.turnTurret(ShooterSubsystem.llSteer);
      }
     }
      else{
       ShooterSubsystem.turnTurret(0);
       //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
     }
     

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
