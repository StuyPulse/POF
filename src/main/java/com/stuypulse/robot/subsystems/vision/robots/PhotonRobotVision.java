package com.stuypulse.robot.subsystems.vision.robots;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.DefenseBotSim;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class PhotonRobotVision extends RobotVision{

    private final PhotonCamera camera;

    public PhotonRobotVision() {
        this.camera = new PhotonCamera(Cameras.DRIVER_CAMERA.getName());
    }

    private Translation2d targetToRobotRelativeTranslation2d(PhotonTrackedTarget target) {
        Pose3d offset = Cameras.DRIVER_CAMERA.getLocation();
        double yaw = Units.degreesToRadians(target.getYaw());
        double pitch = Units.degreesToRadians(-target.getPitch());

        // these are not scaled yet to the correct distance from the camera
        double cameraToRobotX = (1/Math.tan(offset.getRotation().getY() - pitch))* offset.getZ();
        double cameraToRobotY = Math.tan(yaw-offset.getRotation().getZ())*cameraToRobotX;

        // scaling step
        double cameraDistanceToRobot = PhotonUtils.calculateDistanceToTargetMeters(
            Cameras.DRIVER_CAMERA.getLocation().getZ(), 
            0, 
            -Cameras.DRIVER_CAMERA.getLocation().getRotation().getY(), 
            pitch
        );
        Translation2d cameraToRobot = new Vector2D(cameraToRobotX, cameraToRobotY).normalize().mul(cameraDistanceToRobot).getTranslation2d();
        
        Translation2d robotToRobot = cameraToRobot.plus(new Translation2d(offset.getX(), offset.getY()));
        return robotToRobot;
    }

    @Override
    public List<Pair<Translation2d, Translation2d>> getRobotPositionsAsBoundingBoxes() {
        return Robot.isReal() ? getRealRobotPositionsAsBoundingBoxes() : getSimRobotPositionsAsBoundingBoxes();
    }

    private List<Pair<Translation2d, Translation2d>> getRealRobotPositionsAsBoundingBoxes() {
        ArrayList<Translation2d> robotPositions= new ArrayList<Translation2d>();
        camera.getLatestResult().getTargets().forEach(
            (PhotonTrackedTarget target) ->  {
                robotPositions.add(SwerveDrive.getInstance().getPose().transformBy(new Transform2d(targetToRobotRelativeTranslation2d(target), new Rotation2d())).getTranslation());
            }
        );
        
        ArrayList<Pair<Translation2d, Translation2d>> boundingBoxes = new ArrayList<Pair<Translation2d, Translation2d>>();

        for (Translation2d robot : robotPositions) {
            Translation2d corner1 = new Translation2d(
                robot.getX() + (Settings.RobotDetection.averageRobotSideLength / 2), 
                robot.getY() + (Settings.RobotDetection.averageRobotSideLength / 2)
            );
            Translation2d corner2 = new Translation2d(
                robot.getX() - (Settings.RobotDetection.averageRobotSideLength / 2), 
                robot.getY() - (Settings.RobotDetection.averageRobotSideLength / 2)
            );
            Pair<Translation2d, Translation2d> boundingBox = new Pair<Translation2d,Translation2d>(corner1, corner2);
            boundingBoxes.add(boundingBox);
        }

        return boundingBoxes;
    }

    private List<Pair<Translation2d, Translation2d>> getSimRobotPositionsAsBoundingBoxes() {
        DefenseBotSim defenseBot = DefenseBotSim.getInstance();
        Translation2d defenseBotPose = defenseBot.getPose().getTranslation();

        ArrayList<Translation2d> defenseBotProjections = new ArrayList<Translation2d>();
        defenseBotProjections.add(defenseBotPose);
        for (double i = 0; i < 0.7; i+=0.1) {
            defenseBotProjections.add(defenseBotPose.plus(new Translation2d(defenseBot.getVelocityX() * i, defenseBot.getVelocityY() * i)));
        }

        ArrayList<Pair<Translation2d, Translation2d>> boundingBoxes = new ArrayList<Pair<Translation2d,Translation2d>>();

        for (Translation2d robot : defenseBotProjections) {
            Translation2d corner1 = new Translation2d(
                robot.getX() + (Settings.RobotDetection.averageRobotSideLength / 2), 
                robot.getY() + (Settings.RobotDetection.averageRobotSideLength / 2)
            );
            Translation2d corner2 = new Translation2d(
                robot.getX() - (Settings.RobotDetection.averageRobotSideLength / 2), 
                robot.getY() - (Settings.RobotDetection.averageRobotSideLength / 2)
            );
            Pair<Translation2d, Translation2d> boundingBox = new Pair<Translation2d,Translation2d>(corner1, corner2);
            boundingBoxes.add(boundingBox);
        }
        return boundingBoxes;
    }
}
