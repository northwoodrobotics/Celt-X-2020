package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.ShooterSubsystem;



public class SpinUp extends CommandBase {
  // The subsystem the command runs on
  private final ShooterSubsystem spinUp;

  public SpinUp(ShooterSubsystem subsystem) {
    spinUp = subsystem;
    addRequirements(spinUp);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
      
    ShooterSubsystem.spinBooster(4000);
    ShooterSubsystem.spinShooter(4800);
    ShooterSubsystem.compressorDisabled();

  }

  @Override
  public boolean isFinished() {
    
    return true;

  }

  @Override
  public void end(boolean interrupted) {

    ShooterSubsystem.stopBooster();
    ShooterSubsystem.stopShooter();
    ShooterSubsystem.compressorEnabled();

  }
}