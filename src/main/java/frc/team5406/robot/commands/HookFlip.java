package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class HookFlip extends CommandBase {
    // The subsystem the command runs on
    private final ClimbSubsystem climb;

    public HookFlip(ClimbSubsystem subsystem) {
        climb = subsystem;
        addRequirements(climb);
    }

    @Override
    public void initialize() {

        ClimbSubsystem.setupMotors();

    }

    @Override
    public void execute() {

        ClimbSubsystem.releaseBreak();
        ClimbSubsystem.setPosition(ClimbSubsystem.getPosition() + 1);
        new WaitCommand(1.5);
        ClimbSubsystem.setPosition(ClimbSubsystem.getPosition() - 0.1);

    }

    @Override
    public boolean isFinished() {

        return true;
    }

    @Override
    public void end(boolean interrupted) {

        ClimbSubsystem.setSpeed(0);
        ClimbSubsystem.setBrake();

    }
}