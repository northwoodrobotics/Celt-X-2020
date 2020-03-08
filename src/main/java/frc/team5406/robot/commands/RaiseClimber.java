package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ClimbSubsystem;


public class RaiseClimber extends CommandBase {
  // The subsystem the command runs on
  private final ClimbSubsystem climb;

  public RaiseClimber(ClimbSubsystem subsystem) {
    climb = subsystem;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    
    ClimbSubsystem.setupMotors();

  }

  @Override
  public void execute() {
      
    ClimbSubsystem.releaseBreak();
    ClimbSubsystem.setSpeed(Constants.CLIMBER_MOTOR);
    ClimbSubsystem.setPosition(Constants.CLIMBER_UP_POSITION);  // Not in Consatnts yet

  }

  @Override
  public boolean isFinished() {
    
    return true;
  }

  @Override
  public void end(boolean interrupted) {

  }
}