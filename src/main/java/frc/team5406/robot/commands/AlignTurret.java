package frc.team5406.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AlignTurret extends CommandBase {

    // The subsystem the command runs on
  private final ShooterSubsystem shooter;

  public AlignTurret(ShooterSubsystem subsystem) {
    shooter = subsystem;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    
    ShooterSubsystem.updateLimelightTracking();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

  }

  @Override
  public void execute() {
    ShooterSubsystem.updateLimelightTracking();

    if (ShooterSubsystem.llHasValidTarget) {

        ShooterSubsystem.adjustTurret(ShooterSubsystem.llSteer);
    }

  }

  @Override
  public boolean isFinished() {
    return (ShooterSubsystem.llHasValidTarget && Math.abs(NetworkTableInstance.getDefault()
    .getTable("limelight").getEntry("tx").getDouble(0)) < 1);

    

  }

  @Override
  public void end(boolean interrupted) {

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

  }
}