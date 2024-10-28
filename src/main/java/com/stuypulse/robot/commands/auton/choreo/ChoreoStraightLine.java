package com.stuypulse.robot.commands.auton.choreo;

import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoStraightLine extends SequentialCommandGroup {
    public ChoreoStraightLine(ChoreoTrajectory... trajectories) {
        addCommands(
            SwerveDrive.getInstance().choreoSwervePath(trajectories[0])
        );
    }
}