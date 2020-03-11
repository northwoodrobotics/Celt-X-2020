package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AdjustTurret extends CommandBase {

  // The subsystem the command runs on
  private final ShooterSubsystem shooter;
  DoubleSupplier m_rotation;

  public AdjustTurret(ShooterSubsystem subsystem, DoubleSupplier rotation) {

    shooter = subsystem;
    m_rotation = rotation;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    ShooterSubsystem.turnTurret(m_rotation.getAsDouble() * 0.5);
  }

  @Override
  public boolean isFinished() {

    if (m_rotation.getAsDouble() == 0) {

    }

    return true;
  }

  @Override
  public void end(boolean interrupted) {

    ShooterSubsystem.turnTurret(0);
  }
}