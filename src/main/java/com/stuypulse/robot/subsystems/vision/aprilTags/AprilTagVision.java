package com.stuypulse.robot.subsystems.vision.aprilTags;

import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public abstract class AprilTagVision extends SubsystemBase {

    private static final AprilTagVision instance;

    static {
        instance = new PhotonTagVision();
    }

    public static AprilTagVision getInstance() {
        return instance;
    }

    public abstract ArrayList<VisionData> getOutputs();

    public abstract void setTagWhitelist(int... ids);

    public abstract void setCameraEnabled(String name, boolean enabled);
}
