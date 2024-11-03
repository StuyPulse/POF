package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.PhotonTagVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionChangeWhiteList extends InstantCommand {

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private final int[] ids;

    public VisionChangeWhiteList(int... ids) {
        this.ids = ids;
    }

    @Override
    public void initialize() {
        PhotonTagVision.getInstance().setTagWhitelist(ids);
    }
}
