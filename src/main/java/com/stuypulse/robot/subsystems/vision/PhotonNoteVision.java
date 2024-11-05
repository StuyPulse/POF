package com.stuypulse.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
            Pose3d offset = Cameras.NOTE_CAMERA.getLocation();
            double noteYaw = Units.degreesToRadians(note.getYaw());
            double notePitch = Units.degreesToRadians(note.getPitch());

            // these are not scaled yet to the correct distance from the camera
            double noteX = (1/Math.tan(notePitch-offset.getRotation().getY()))* offset.getZ();
            double noteY = Math.tan(noteYaw-offset.getRotation().getZ())*noteX;

            // scaling step
            double distanceToNote = PhotonUtils.calculateDistanceToTargetMeters(
                Cameras.NOTE_CAMERA.getLocation().getZ(), 
                0, 
                -Cameras.NOTE_CAMERA.getLocation().getRotation().getY(), 
                notePitch
            );
            Translation2d cameraToNote = new Vector2D(noteX, noteY).normalize().mul(distanceToNote).getTranslation2d();

            Translation2d notePose = cameraToNote.plus(new Translation2d(offset.getX(), offset.getY()));
            if (notePose.getNorm() < closestNoteDistance) {
                closestNoteDistance = notePose.getNorm();
                closestRobotRelativeNotePose = notePose;
            }
        }
        return closestRobotRelativeNotePose;
    }
    
}
