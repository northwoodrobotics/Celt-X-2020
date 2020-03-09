package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AutoTurnTurret extends CommandBase {

    // The subsystem the command runs on
    private final ShooterSubsystem shooter;
    private final double turretAngle;

  public AutoTurnTurret(ShooterSubsystem subsystem, double angle) {
    shooter = subsystem;
    turretAngle = angle;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    

  }

  @Override
  public void execute() {
      
    ShooterSubsystem.turnTurret(turretAngle);
  }

  @Override
  public boolean isFinished() {
    //FIX - should wait until turret reaches target
    return true;
  }

  @Override
  public void end(boolean interrupted) {


  }
}