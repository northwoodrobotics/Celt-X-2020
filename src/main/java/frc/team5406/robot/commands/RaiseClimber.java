package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class RaiseClimber extends CommandBase {
  // The subsystem the command runs on
  private final ClimbSubsystem climb;
  double position = 0;
  public RaiseClimber(ClimbSubsystem subsystem, double _position) {
    climb = subsystem;
    position = _position;
    addRequirements(climb);
  }

  @Override
  public void initialize() {

    ClimbSubsystem.setPosition(position);

  }

  @Override
  public void execute() {


  }

  @Override
  public boolean isFinished() {
    

    return -1*ClimbSubsystem.getClimberPosition() > -1*position ;
  }

  @Override
  public void end(boolean interrupted) {


  }
}