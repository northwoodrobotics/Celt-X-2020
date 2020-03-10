package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;



public class AutoShootAndIntake extends CommandBase {
  // The subsystem the command runs on
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private boolean readyToShoot = false;

  public AutoShootAndIntake(ShooterSubsystem _shooter, IntakeSubsystem _intake) {
    shooter = _shooter;
    intake = _intake;
    addRequirements(shooter);
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    
    ShooterSubsystem.spinShooterAuto();
    ShooterSubsystem.setHoodAngleAuto();
    ShooterSubsystem.spinBooster(Constants.BOOSTER_OUTPUT);
    ShooterSubsystem.compressorDisabled();
    IntakeSubsystem.intakeExtend();   
    IntakeSubsystem.spinRollers();
    ShooterSubsystem.updateLimelightTracking();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);



  }

  @Override
  public void execute() {
    ShooterSubsystem.updateLimelightTracking();

    if (ShooterSubsystem.llHasValidTarget) {

        ShooterSubsystem.adjustTurret(ShooterSubsystem.llSteer);
    }

      
    if(ShooterSubsystem.checkRPM() && !readyToShoot) {
      
      readyToShoot = true;
    }
    if(readyToShoot) {

      ShooterSubsystem.spinFeeder(1000);
      IntakeSubsystem.serialize();
      

    }

  }

  @Override
  public boolean isFinished() {
    
    return false;
  }

  @Override
  public void end(boolean interrupted) {

    ShooterSubsystem.compressorEnabled();
    ShooterSubsystem.stopShooter();
    ShooterSubsystem.stopBooster();
    ShooterSubsystem.setHoodAngle(0);
    ShooterSubsystem.stopFeeder();
    IntakeSubsystem.stopSerialize();
    IntakeSubsystem.intakeRetract();
    IntakeSubsystem.stopIntake();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

  }
}