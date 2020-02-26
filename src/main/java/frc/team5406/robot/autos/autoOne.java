package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.team5406.robot.subsystems.DriveSubsystem;

public class autoOne {

    private final DriveSubsystem drive = new Drive();

    XboxController driverGamepad = new XboxController(1);

    public autoOne () {
        
        drive.setDefaultCommand(
        
        new RunCommand(() -> drive
            .arcadeDrive(driverGamepad.getY(GenericHID.Hand.kLeft),
                         driverGamepad.getX(GenericHID.Hand.kRight)), drive));
    }

    public Command getAutonomousCommand() {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.S_VOLTS,
                                       Constants.V_VOLTS,
                                       Constants.A_VOLTS),
            Constants.DRIVE_KINEMATICS,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);

        /* Rotation2d endpoint = new Rotation2d();
        endpoint = Rotation2d.fromDegrees(45); */

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            //new Translation2d(1, 1),
            //new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config
    );
    
        RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drive::getPose,
        new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
        Constants.DRIVE_KINEMATICS,
        drive::outputSpeeds,
        drive
    );


        return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
      }
    
}

