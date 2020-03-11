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
import frc.team5406.robot.autos.DriveStraight;
import frc.team5406.robot.autos.FiveBallLeft;
import frc.team5406.robot.autos.SevenBallLeft;
import frc.team5406.robot.autos.EightBallRight;
import frc.team5406.robot.autos.SixBallRight;
import frc.team5406.robot.autos.ThreeBallCenter;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;
  private DriveStraight driveStraight;
  private FiveBallLeft fiveBallLeft;
  private SevenBallLeft sevenBallLeft;
  private ThreeBallCenter threeBallCenter;
  private EightBallRight eightBallRight;
  private SixBallRight sixBallRight;
  private static final String driveStraightString = "DriveStraight";
  private static final String fiveBallLeftString = "FiveBallLeft";
  private static final String sevenBallLeftString = "SevenBallLeft";
  public static final String threeBallCenterString = "ThreeBallCenter";
  private static final String eightBallRightString = "EightBallRight";
  private static final String sixBallRightString = "SixBallRight";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  XboxController operatorGamepad = new XboxController(Constants.OPERATOR_CONTROLLER);
  XboxController driverGamepad = new XboxController(Constants.DRIVER_CONTROLLER);
  private DriveSubsystem robotDrive = new DriveSubsystem();
  int intakePulseCount = 0;
  boolean baselock;

  boolean dPadPressed = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    driveStraight = new DriveStraight();
    fiveBallLeft = new FiveBallLeft();
    sevenBallLeft = new SevenBallLeft();
    threeBallCenter = new ThreeBallCenter();
    eightBallRight = new EightBallRight();
    sixBallRight = new SixBallRight();
    // m_robotContainer = new RobotContainer();
    ShooterSubsystem.setupMotors();
    IntakeSubsystem.setupMotors();
    ClimbSubsystem.setupMotors();

    m_chooser.setDefaultOption("Drive Straight Backwards", driveStraightString);
    m_chooser.addOption("Five Ball, Left", fiveBallLeftString);
    m_chooser.addOption("Seven Ball, Left", sevenBallLeftString);
    m_chooser.addOption("Three Ball, Center", threeBallCenterString);
    m_chooser.addOption("Eight Ball, Right", eightBallRightString);
    m_chooser.addOption("Six Ball, Right", sixBallRightString);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
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
    DriveSubsystem.setBrakeMode(false);
  }

  @Override
  public void disabledPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    IntakeSubsystem.djSpinnerDown();
    DriveSubsystem.setBrakeMode(true);
    if (m_autoSelected == null) {
      m_autoSelected = "none";
    }

    switch (m_autoSelected) {

    case fiveBallLeftString:
      m_autonomousCommand = fiveBallLeft.getAutonomousCommand();
      break;
    case sevenBallLeftString:
      m_autonomousCommand = sevenBallLeft.getAutonomousCommand();
      break;
    case sixBallRightString:
      m_autonomousCommand = sixBallRight.getAutonomousCommand();
      break;
    case threeBallCenterString:
      m_autonomousCommand = threeBallCenter.getAutonomousCommand();
      break;
    case eightBallRightString:
      m_autonomousCommand = eightBallRight.getAutonomousCommand();
      break;
    case driveStraightString:
    default:
      m_autonomousCommand = driveStraight.getAutonomousCommand();
      break;
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

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
    DriveSubsystem.setBrakeMode(false);

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    SmartDashboard.putNumber("Shooter Multiplier", ShooterSubsystem.shooterMultiplier);
    SmartDashboard.putNumber("ts", ts);
    SmartDashboard.putNumber("tx", tx);

    // if((driverGamepad.getY(Hand.kLeft) > Constants.JOYSTICK_DEADBAND ||
    // (driverGamepad.getX(Hand.kRight) > Constants.JOYSTICK_DEADBAND))) {

    robotDrive.arcadeDrive(driverGamepad.getY(Hand.kLeft), driverGamepad.getX(Hand.kRight));
    /*
     * baselock = false;
     * 
     * } else if(driverGamepad.getY(Hand.kLeft) <= Constants.JOYSTICK_DEADBAND &&
     * (driverGamepad.getX(Hand.kRight) <= Constants.JOYSTICK_DEADBAND) &&
     * (Math.abs(DriveSubsystem.getAverageSpeed()) >=
     * Constants.STATIONARY_SPEED_THRESHOLD)) {
     * 
     * DriveSubsystem.stopMotors();
     * 
     * } else if(!baselock && (Math.abs(DriveSubsystem.getAverageSpeed()) <
     * Constants.STATIONARY_SPEED_THRESHOLD)) {
     * 
     * DriveSubsystem.baselock(); baselock = true; }
     */
    if (operatorGamepad.getBumper(Hand.kRight) && operatorGamepad.getStartButtonPressed()) {
      ClimbSubsystem.setBrake();
    }

    if (operatorGamepad.getBumper(Hand.kRight) && operatorGamepad.getBackButtonPressed()) {
      ClimbSubsystem.releaseBreak();

    }
    if (operatorGamepad.getBumper(Hand.kRight) && (Math.abs(operatorGamepad.getY(Hand.kLeft)) > 0.15)) {
      double leftSpeed = operatorGamepad.getY(Hand.kLeft);
      ClimbSubsystem.setSpeed(leftSpeed);
    } else {
      ClimbSubsystem.setSpeed(0);
    }

    SmartDashboard.putNumber("Shooter RPM", ShooterSubsystem.getShooterSpeed());
    SmartDashboard.putNumber("Feeder RPM", ShooterSubsystem.getBoosterSpeed());
    SmartDashboard.putNumber("Hood Output", ShooterSubsystem.getHoodVoltage());
    SmartDashboard.putNumber("Feeder RPM", ShooterSubsystem.getFeederSpeed());
    SmartDashboard.putNumber("Abs Hood", ShooterSubsystem.getAbsHoodPosition());
    if (operatorGamepad.getXButton() && !operatorGamepad.getBumper(Hand.kRight)) {
      IntakeSubsystem.djSpinnerUp();
      IntakeSubsystem.spinDJSpinner();
    } else if (operatorGamepad.getYButton() && !operatorGamepad.getBumper(Hand.kRight)) {
      IntakeSubsystem.djSpinnerUp();
    } else {
      IntakeSubsystem.djSpinnerDown();
      IntakeSubsystem.stopDJSpinner();
    }
    if (operatorGamepad.getBButton() && !operatorGamepad.getBumper(Hand.kRight)) {
      //ShooterSubsystem.spinShooter(SmartDashboard.getNumber("Shooter Target RPM", 5000));
      ShooterSubsystem.spinBooster(SmartDashboard.getNumber("Booster Target RPM", 4000));
     // ShooterSubsystem.setHoodAngle(SmartDashboard.getNumber("Hood Target Angle", 0));
      ShooterSubsystem.compressorDisabled();
       ShooterSubsystem.spinShooterAuto();
       ShooterSubsystem.setHoodAngleAuto();
      if (ShooterSubsystem.checkRPM()) {
        operatorGamepad.setRumble(RumbleType.kLeftRumble, 1);

      } else {
        operatorGamepad.setRumble(RumbleType.kLeftRumble, 0);
      }

      // This is Test Mode for Pit
    } else if (operatorGamepad.getBButton() && operatorGamepad.getBumper(Hand.kRight)) {
      ShooterSubsystem.spinShooter(1200);
      ShooterSubsystem.spinBooster(6500);
      ShooterSubsystem.setHoodAngle(40);
      ShooterSubsystem.compressorDisabled();

    }
    // pre-spin the shooter wheels
    else if (operatorGamepad.getTriggerAxis(Hand.kLeft) > .1) {
      ShooterSubsystem.spinShooter(4800);
      ShooterSubsystem.spinBooster(4000);

      ShooterSubsystem.compressorDisabled();
    } else {
      ShooterSubsystem.compressorEnabled();
      ShooterSubsystem.stopShooter();
      ShooterSubsystem.stopBooster();
      ShooterSubsystem.setHoodAngle(0);
      operatorGamepad.setRumble(RumbleType.kLeftRumble, 0);
      // ShooterSubsystem.releaseHood();
    }

    if (driverGamepad.getTriggerAxis(Hand.kRight) > .1) {
      IntakeSubsystem.spinRollers();
    } else if (!(operatorGamepad.getTriggerAxis(Hand.kRight) > .1)) {
      IntakeSubsystem.stopRollers();
    }

    if (driverGamepad.getBumper(Hand.kRight)) {
      IntakeSubsystem.intakeExtend();
    } else {
      IntakeSubsystem.intakeRetract();
    }

    if (operatorGamepad.getTriggerAxis(Hand.kRight) > .1) {
      IntakeSubsystem.pulseRollers();
      ShooterSubsystem.spinFeeder(1000);
      // ShooterSubsystem.compressorDisabled();
      if (!(driverGamepad.getTriggerAxis(Hand.kRight) > .1)) {
        IntakeSubsystem.serialize();
      }

    } else if (!(driverGamepad.getTriggerAxis(Hand.kLeft) > .1)) {
      ShooterSubsystem.stopFeeder();
      // ShooterSubsystem.compressorEnabled();
      if (!(driverGamepad.getTriggerAxis(Hand.kRight) > .1)) {
        IntakeSubsystem.stopSerialize();
      }

    }

    if (driverGamepad.getTriggerAxis(Hand.kLeft) > .1) {
      IntakeSubsystem.reverseIntake();
      IntakeSubsystem.reverseSerialize();
      ShooterSubsystem.reverseFeeder();
    }

    if ((Math.abs(operatorGamepad.getX(Hand.kRight)) > 0.1) && (operatorGamepad.getBumper(Hand.kRight))) {
      ShooterSubsystem.adjustTurret(operatorGamepad.getX(Hand.kRight));
    } else if (operatorGamepad.getAButton()) {
      ShooterSubsystem.updateLimelightTracking();
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      ShooterSubsystem.LLAlignTurret();
    } else if (operatorGamepad.getPOV() == 0 && !dPadPressed) {
      dPadPressed = true;
      ShooterSubsystem.turnTurret(0);
    } else if (operatorGamepad.getPOV() == 90 && !dPadPressed) {
      dPadPressed = true;
      ShooterSubsystem.turnTurret(90);
    } else if (operatorGamepad.getPOV() == 180 && !dPadPressed) {
      dPadPressed = true;
      ShooterSubsystem.turnTurret(180);
    } else if (operatorGamepad.getPOV() == 270 && !dPadPressed) {
      dPadPressed = true;
      ShooterSubsystem.turnTurret(-35);
    }

    else {
      // ShooterSubsystem.adjustTurret(0);
      dPadPressed = false;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
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
