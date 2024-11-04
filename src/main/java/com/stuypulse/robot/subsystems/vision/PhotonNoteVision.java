package com.stuypulse.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
            Translation2d notePose = Cameras.NOTE_CAMERA.getLocation().transformBy(new Transform3d(note.getBestCameraToTarget().getTranslation(), new Rotation3d())).getTranslation().toTranslation2d();
            if (notePose.getNorm() < closestNoteDistance) {
                closestNoteDistance = notePose.getNorm();
                closestRobotRelativeNotePose = notePose;
            }
        }
        return closestRobotRelativeNotePose;
    }
    
}
