package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AutoTurnTurret extends CommandBase {

    // The subsystem the command runs on
  private final ShooterSubsystem shooter;

  public AutoTurnTurret(ShooterSubsystem subsystem) {
    shooter = subsystem;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    

  }

  @Override
  public void execute() {
      
   // ShooterSubsystem.turnTurret(turn);
  }

  @Override
  public boolean isFinished() {
    
    return true;
  }

  @Override
  public void end(boolean interrupted) {

    ShooterSubsystem.turnTurret(0);

  }
}