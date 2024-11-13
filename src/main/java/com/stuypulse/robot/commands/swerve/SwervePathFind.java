package com.stuypulse.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePathFind{
    
    public static Command toPose(Pose2d pose) {
        Command command = AutoBuilder.pathfindToPose(pose, Settings.Swerve.Motion.DEFAULT_CONSTRAINTS)
            .until(() -> SwerveDrive.getInstance().getPose().getTranslation().getDistance(pose.getTranslation()) < 1.0);

        command.addRequirements(SwerveDrive.getInstance());
        return command;
    }

    // TODO find actual optimal point to drive to
    public static Command toShoot() {
        return toPose(Field.getAllianceSpeakerPose().transformBy(new Transform2d(2, 0, new Rotation2d())));
    }

    // just an arbitrary point for testing
    public static Command toPickup() {
        return toPose(new Pose2d(
            Robot.isBlue() ? 3 : Field.LENGTH - 3, 
            1,
            Robot.isBlue() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180))
        );
    }

    public static Command test() {
        return toPose(new Pose2d(Field.getAmpCornerPose(), new Rotation2d())).andThen(toPose(new Pose2d(6.0, Field.WIDTH - 1.25, new Rotation2d()))).repeatedly();
    }
}
