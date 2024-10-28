package com.stuypulse.robot.commands.auton.choreo;

import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoSquareSplit extends SequentialCommandGroup {
    
    
    
    public ChoreoSquareSplit(ChoreoTrajectory... trajectories) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        addCommands(
            swerve.choreoSwervePath(trajectories[0]),
            swerve.choreoSwervePath(trajectories[1])
        );
    }
}
