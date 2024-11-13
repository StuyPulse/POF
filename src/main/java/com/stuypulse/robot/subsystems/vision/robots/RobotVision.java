package com.stuypulse.robot.subsystems.vision.robots;

import java.util.List;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.constants.Ports.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class RobotVision extends SubsystemBase{

    private static RobotVision instance;

    static {
        instance = new PhotonRobotVision();
    }

    public static RobotVision getInstance() {
        return instance;
    }

    public abstract List<Pair<Translation2d, Translation2d>> getRobotPositionsAsBoundingBoxes();

    private Translation2d average(Translation2d pose1, Translation2d pose2) {
        return new Translation2d((pose1.getX() + pose2.getX()) / 2, (pose1.getY() + pose2.getY()) / 2);
    }

    @Override
    public void periodic() {
        List<Pair<Translation2d, Translation2d>> boundingBoxes = getRobotPositionsAsBoundingBoxes();
        Pathfinding.setDynamicObstacles(boundingBoxes, SwerveDrive.getInstance().getPose().getTranslation());
        if (boundingBoxes.size() > 0) {
            Translation2d cone = average(boundingBoxes.get(0).getFirst(), boundingBoxes.get(0).getSecond());
            SmartDashboard.putNumber("Robot Detection/Cone X", cone.getX());
            SmartDashboard.putNumber("Robot Detection/Cone Y", cone.getY());
        }

    }
}
