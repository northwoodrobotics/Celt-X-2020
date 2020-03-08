package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;


public class AdjustClimber extends CommandBase {
  // The subsystem the command runs on
  private final ClimbSubsystem climb;

  public AdjustClimber(ClimbSubsystem subsystem) {
    climb = subsystem;
    addRequirements(climb);
  }
 
  XboxController operatorGamepad = new XboxController(Constants.OPERATOR_CONTROLLER);

  @Override
  public void initialize() {
    
    ClimbSubsystem.setupMotors();

  }

  @Override
  public void execute() {
      
    double leftSpeed = operatorGamepad.getY(Hand.kLeft);
    ClimbSubsystem.setPosition(leftSpeed);

  }

  @Override
  public boolean isFinished() {
    
    return true;
  }

  @Override
  public void end(boolean interrupted) {

  }
}