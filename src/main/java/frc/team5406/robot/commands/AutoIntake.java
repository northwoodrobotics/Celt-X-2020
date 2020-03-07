package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.IntakeSubsystem;


public class AutoIntake extends CommandBase {
  // The subsystem the command runs on
  private final IntakeSubsystem intake;

  public AutoIntake(IntakeSubsystem subsystem) {
    intake = subsystem;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    
    IntakeSubsystem.intakeExtend();
  }

  @Override
  public void execute() {
      
    IntakeSubsystem.spinRollers();

  }

  @Override
  public boolean isFinished() {

    // SOMETHING GOES IN HERE
    
    return true;
  }

  @Override
  public void end(boolean interrupted) {

    IntakeSubsystem.intakeRetract();
    IntakeSubsystem.stopIntake();
  }
}