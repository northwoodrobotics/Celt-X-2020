package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;


public class AutoShoot extends CommandBase {
  // The subsystem the command runs on
  private final ShooterSubsystem shooter;
  private boolean readyToShoot = false;

  public AutoShoot(ShooterSubsystem subsystem) {
    shooter = subsystem;
    addRequirements(shooter);
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