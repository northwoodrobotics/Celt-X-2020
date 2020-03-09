package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;


public class AutoShoot extends CommandBase {
  // The subsystem the command runs on
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private boolean readyToShoot = false;

  public AutoShoot(ShooterSubsystem _shooter, IntakeSubsystem _intake) {
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

  }

  @Override
  public void execute() {
      
    if(ShooterSubsystem.checkRPM() && !readyToShoot) {
      
      readyToShoot = true;
    }
    if(readyToShoot) {

      ShooterSubsystem.spinFeeder(1000);
      IntakeSubsystem.pulseRollers();
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

  }
}