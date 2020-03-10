package frc.team5406.robot.autos;

import frc.team5406.robot.Constants;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.CANPIDController;

import frc.team5406.robot.subsystems.DriveSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.commands.SpinUp;
import frc.team5406.robot.commands.AutoTurnTurret;
import frc.team5406.robot.commands.AlignTurret;
import frc.team5406.robot.commands.AutoIntake;
import frc.team5406.robot.commands.AutoShoot;
import frc.team5406.robot.commands.AutoShootAndIntake;

public class FiveBallLeft {

    private final DriveSubsystem drive = new DriveSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();


    public FiveBallLeft () {
        
    }

    public Command getAutonomousCommand() {
        drive.reset();
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.S_VOLTS,
                                       Constants.V_VOLTS,
                                       Constants.A_VOLTS),
            Constants.DRIVE_KINEMATICS,
            10);


    TrajectoryConfig config1 =
        new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                             Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            .setKinematics(Constants.DRIVE_KINEMATICS)
            .addConstraint(autoVoltageConstraint);

    Trajectory drive1 = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(1,0, new Rotation2d(0)),
            new Pose2d(2.80, -0.45, new Rotation2d(Units.degreesToRadians(-20)))

        ),
        config1
    );

    TrajectoryConfig config2 =
    new TrajectoryConfig(Constants.MAX_SPEED_METERS_PER_SECOND,
                         Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(Constants.DRIVE_KINEMATICS)
        .addConstraint(autoVoltageConstraint)
        .setReversed(true);


Trajectory drive2 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(2.80, -0.45, new Rotation2d(Units.degreesToRadians(-20))),
    List.of(),
    new Pose2d(0.8, 3.5, new Rotation2d(Units.degreesToRadians(-25))),
    config2
);



        return   new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new RamseteCommand(
                    drive1,
                    drive::getPose,
                    new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                    Constants.DRIVE_KINEMATICS,
                    drive::outputSpeeds,
                    drive
                    ).andThen(() -> drive.tankDriveVolts(0, 0))
                ,new AutoIntake(intake)
            )
            ,new SpinUp(shooter)
            ,new ParallelCommandGroup(
                new RamseteCommand(
                    drive2,
                    drive::getPose,
                    new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                    Constants.DRIVE_KINEMATICS,
                    drive::outputSpeeds,
                    drive
                    ).andThen(() -> drive.tankDriveVolts(0, 0))
                ,new AutoTurnTurret(shooter, 160)
            )
            ,new AlignTurret(shooter)
            ,new AutoShoot(shooter, intake)
        );

    }
    
}

