package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;


public class Unjam extends CommandBase {
  // The subsystem the command runs on
  private final IntakeSubsystem outtake;
  private final ShooterSubsystem revFeeder;

  public Unjam(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    outtake = intakeSubsystem;
    revFeeder = shooterSubsystem;
    addRequirements(outtake);
    addRequirements(revFeeder);
  }

  @Override
  public void initialize() {
    
    IntakeSubsystem.intakeExtend();
  }

  @Override
  public void execute() {
      
    IntakeSubsystem.reverseIntake();
    IntakeSubsystem.reverseSerialize();
    ShooterSubsystem.reverseFeeder();

  }

  @Override
  public boolean isFinished() {
    
    return true;
  }

  @Override
  public void end(boolean interrupted) {

    IntakeSubsystem.intakeRetract();
    IntakeSubsystem.stopIntake();
    ShooterSubsystem.stopFeeder();
  }
}