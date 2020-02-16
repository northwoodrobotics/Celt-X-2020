package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.team5406.robot.subsystems.IntakeSubsystem;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}.  Written explicitly for
 * pedagogical purposes.  Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */

public class SpinIntake extends CommandBase {
  // The subsystem the command runs on
  private final IntakeSubsystem intake;

  public SpinIntake(IntakeSubsystem subsystem) {
    intake = subsystem;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.spinBrushes();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}