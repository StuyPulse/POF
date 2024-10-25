package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveDriveDriveAlignedSpeaker extends SwerveDriveDriveAligned{
    
    public SwerveDriveDriveAlignedSpeaker(Gamepad driver) {
        super(driver);
    }

    private Pose2d getTargetSpeakerPose() {
        Pose2d speakerPose = Field.getAllianceSpeakerPose().transformBy(new Transform2d(-Units.inchesToMeters(1.5), 0, new Rotation2d()));
        Pose2d robotPose = SwerveDrive.getInstance().getPose();

        double angleFromSpeakerBaseToRobot = Units.radiansToDegrees(Math.atan((speakerPose.getY() - robotPose.getY())/(speakerPose.getX() - robotPose.getX())));

        // aim at the side of the speaker if youre on the side
        // if (angleFromSpeakerBaseToRobot > 30) {
        //     speakerPose = speakerPose.transformBy(new Transform2d(0, Field.SPEAKER_OPENING_WIDTH / 2, new Rotation2d()));
        // }
        // if (angleFromSpeakerBaseToRobot < 30) {
        //     speakerPose = speakerPose.transformBy(new Transform2d(0, -Field.SPEAKER_OPENING_WIDTH / 2, new Rotation2d()));
        // }
        return speakerPose;
    }

    @Override
    protected Rotation2d getTargetAngle() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = getTargetSpeakerPose().getTranslation();
        return currentPose.minus(speakerPose).getAngle();
    }

    @Override
    protected double getDistanceToTarget() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = getTargetSpeakerPose().getTranslation();
        return currentPose.getDistance(speakerPose);
    }
}
