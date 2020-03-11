package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.IntakeSubsystem;

public class TeleopIntake extends CommandBase {

  private final IntakeSubsystem intake;
  private final DoubleSupplier m_buttonPress;

  public TeleopIntake(IntakeSubsystem subsystem, DoubleSupplier buttonPress) {
    intake = subsystem;
    m_buttonPress = buttonPress;
    addRequirements(intake);
  }

  @Override
  public void initialize() {

    IntakeSubsystem.intakeExtend();
  }

  @Override
  public void execute() {
    // please change the one later
    if (m_buttonPress.getAsDouble() == 1) {

      IntakeSubsystem.spinRollers();
    }

  }

  @Override
  public boolean isFinished() {

    // SOMETHING GOES IN HERE

    return false;
  }

  @Override
  public void end(boolean interrupted) {

    IntakeSubsystem.intakeRetract();
    IntakeSubsystem.stopIntake();
  }

}