// release brake
// 1) go to predefined height  (set pos something level height) 

// 2) manual

// 3) get down (pos 5 set brake)

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.Constants;

public class AutoClimb extends CommandBase {
    // The subsystem the command runs on
    private final ClimbSubsystem climb;

    XboxController operatorGamepad = new XboxController(Constants.OPERATOR_CONTROLLER);

    public AutoClimb(ClimbSubsystem subsystem) {
        climb = subsystem;
        addRequirements(climb);
    }

    @Override
    public void initialize() {

        ClimbSubsystem.setupMotors();

    }

    @Override
    public void execute() {

        if (operatorGamepad.getBumper(Hand.kRight) && operatorGamepad.getStartButtonPressed()) {
            ClimbSubsystem.releaseBreak();
            ClimbSubsystem.setPosition(Constants.CLIMBER_UP_POSITION);

        }

        if (operatorGamepad.getBumper(Hand.kRight) && operatorGamepad.getBackButtonPressed()) {
            ClimbSubsystem.releaseBreak();
            ClimbSubsystem.setPosition(Constants.CLIMBER_DOWN_POSITION + 5);

        }

        if (operatorGamepad.getBumper(Hand.kRight) && (Math.abs(operatorGamepad.getY(Hand.kLeft)) > 0.15)) {
            double leftSpeed = operatorGamepad.getY(Hand.kLeft);
            ClimbSubsystem.releaseBreak();
            ClimbSubsystem.setSpeed(leftSpeed);
        } else {
            ClimbSubsystem.setSpeed(0);
        }

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
