package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.IntakeSubsystem;


public class GobbleBalls extends CommandBase {
  // The subsystem the command runs on
  private final IntakeSubsystem intake;

  public GobbleBalls(IntakeSubsystem subsystem) {
    intake = subsystem;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    
    IntakeSubsystem.setBrushSpeed(speed);
    IntakeSubsystem.intakeExtend();
  }

  @Override
  public void execute() {
      
    IntakeSubsystem.spinBrushes();
  }

  @Override
  public boolean isFinished() {
    
    return true;
  }

  @Override
  public void end() {

    IntakeSubsystem.intakeRetract();
    IntakeSubsystem.stopIntake();
  }
}