package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class LowerClimber extends CommandBase {
  // The subsystem the command runs on
  private final ClimbSubsystem climb;

  public LowerClimber(ClimbSubsystem subsystem) {
    climb = subsystem;
    addRequirements(climb);
  }

  @Override
  public void initialize() {

    ClimbSubsystem.setupMotors();

  }

  @Override
  public void execute() {

    ClimbSubsystem.setSpeed(Constants.CLIMBER_MOTOR);
    ClimbSubsystem.setPosition(Constants.CLIMBER_DOWN_POSITION);

  }

  @Override
  public boolean isFinished() {

    return true;
  }

  @Override
  public void end(boolean interrupted) {

    ClimbSubsystem.setBrake();

  }
}