package com.stuypulse.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Translation2d;

public class PhotonNoteVision extends NoteVision{

    private final PhotonCamera camera;
    private final SmartBoolean enabled;

    public PhotonNoteVision() {
        this.camera = new PhotonCamera(Cameras.NOTE_CAMERA.getName());
        this.enabled = new SmartBoolean("Note Detection/Enabled", true);
    }

    @Override
    public boolean hasNoteData() {
        return camera.getLatestResult().hasTargets();
    }

    /** Make sure to check if there is note data before calling this! */
    @Override
    public Translation2d getRobotRelativeNotePose() {
        return getClosestRobotRelativeNotePose(camera.getLatestResult());
    }

    /** result should have at least one note, or else it will return (0,0) */
    private Translation2d getClosestRobotRelativeNotePose(PhotonPipelineResult result) {
        double closestNoteDistance = Double.MAX_VALUE;
        Translation2d closestRobotRelativeNotePose = new Translation2d();
        for (PhotonTrackedTarget note : result.targets) {
            if (note.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm() < closestNoteDistance) {
                closestNoteDistance = note.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
                closestRobotRelativeNotePose = note.getBestCameraToTarget().getTranslation().toTranslation2d();
            }
        }
        return closestRobotRelativeNotePose;
    }
    
}
