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
    IntakeSubsystem.spinRollers();

  }

  @Override
  public void execute() {
      

  }

  @Override
  public boolean isFinished() {

    // NEEDS SOME EXTERNAL INPUT OR SEPARATE STOP OUT
    
    return false;
  }

  @Override
  public void end(boolean interrupted) {

    IntakeSubsystem.intakeRetract();
    IntakeSubsystem.stopIntake();
  }
}