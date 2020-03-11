package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.Constants;

public class HookFlip extends CommandBase {
    // The subsystem the command runs on
    private final ClimbSubsystem climb;
    int step = 0;
    double startDelay = 0;

    public HookFlip(ClimbSubsystem subsystem) {
        climb = subsystem;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        System.out.println("HookFlip");

        ClimbSubsystem.releaseBrake();
        ClimbSubsystem.resetPosition();
        System.out.println(ClimbSubsystem.getClimberPosition());

        step = 1;
    }

    @Override
    public void execute() {
       System.out.println("HookFlip " + step);
        System.out.println(ClimbSubsystem.getClimberPosition());
        switch (step) {
        case 1:
            ClimbSubsystem.setPosition(Constants.CLIMB_HOOK_FLIP_UP);
           step++;
            break;
        case 2:
        
            if (ClimbSubsystem.getClimberPosition() < (Constants.CLIMB_HOOK_FLIP_UP + .5)) {
                step++;
            }
            break;
        case 3:
        ClimbSubsystem.setPosition(Constants.CLIMB_HOOK_FLIP_DOWN);
        step++;
            break;
        case 4:
            if (ClimbSubsystem.getClimberPosition() > (Constants.CLIMB_HOOK_FLIP_DOWN - .5)) {
                startDelay = Timer.getFPGATimestamp();
                ClimbSubsystem.setBrake();
                step++;
            }
            break;
        case 5:
            if (Timer.getFPGATimestamp() - startDelay > 0.25) {
                step++;
            }
        }
    }


    @Override
    public boolean isFinished() {

        return (step >= 6);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("HookFlip Done");

        ClimbSubsystem.setSpeed(0);

    }
}