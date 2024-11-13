package com.stuypulse.robot.subsystems.vision.notes;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NoteVision extends SubsystemBase {

    private static final NoteVision instance;

    static {
        instance = new PhotonNoteVision();
    }

    public static NoteVision getInstance() {
        return instance;
    }

    public final boolean withinIntakePath() {
        if (!hasNoteData()) return false;

        Translation2d robotRelative = getRobotRelativeNotePose();

        return robotRelative.getX() > 0
            && robotRelative.getX() < Settings.NoteDetection.INTAKE_THRESHOLD_DISTANCE
            && Math.abs(robotRelative.getY()) < Settings.Swerve.WIDTH / 2.0;
    }

    public abstract boolean hasNoteData();

    public abstract Translation2d getRobotRelativeNotePose();

    public final Rotation2d getRotationToNote() {
        return getRobotRelativeNotePose().getAngle();
    }

    public boolean noteIsUsable() {
        return getRobotRelativeNotePose().getNorm() < Settings.NoteDetection.INTAKE_THRESHOLD_DISTANCE
                && Math.abs(getRotationToNote().getDegrees()) < Settings.NoteDetection.MAX_ANGLE_FROM_CAMERA;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note Detection/Has Note Data", hasNoteData());
        SmartDashboard.putBoolean("Note Detection/Is in Intake Path", withinIntakePath());
        if (hasNoteData()) {
            SmartDashboard.putNumber("Note Detection/Note X", getRobotRelativeNotePose().getX());
            SmartDashboard.putNumber("Note Detection/Note Y", getRobotRelativeNotePose().getY());
            SmartDashboard.putNumber("Note Detection/Note Angle", getRotationToNote().getDegrees());
        }
    }
}
