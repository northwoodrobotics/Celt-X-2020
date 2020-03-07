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
      
    if (ShooterSubsystem.llHasValidTarget) {

        ShooterSubsystem.turnTurret(ShooterSubsystem.llSteer);
    }

  }

  @Override
  public boolean isFinished() {

    if(Math.abs(NetworkTableInstance.getDefault()
    .getTable("limelight").getEntry("tx").getDouble(0)) < 0.1) {

    }  

    return true;
  }

  @Override
  public void end(boolean interrupted) {

    ShooterSubsystem.turnTurret(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

  }
}