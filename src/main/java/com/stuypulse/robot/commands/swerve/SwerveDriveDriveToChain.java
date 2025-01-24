// package com.stuypulse.robot.commands.swerve;

// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
// import com.stuypulse.robot.constants.Field;
// import com.stuypulse.robot.constants.Settings.Alignment;
// import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;

// public class SwerveDriveDriveToChain extends Command {

//     private final SwerveDrive swerve;

//     private final LegacySwerveRequest.FieldCentric drive;

//     private Pose2d trapPose;

//     public SwerveDriveDriveToChain() {
//         swerve = SwerveDrive.getInstance();
//         drive = new LegacySwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {
//         trapPose = Field.getClosestAllianceTrapPose(swerve.getPose());
//     }

//     @Override
//     public void execute() {
//         Rotation2d translationAngle = trapPose.getTranslation().minus(swerve.getPose().getTranslation()).getAngle();
//         Translation2d translation = new Translation2d(Alignment.INTO_CHAIN_SPEED.get(), translationAngle);

//         swerve.setControl(drive.withVelocityX(translation.getX())
//                 .withVelocityY(translation.getY())
//                 .withRotationalRate(0)
//             );
//     }

//     private double getDistanceToTrap() {
//         return swerve.getPose().getTranslation().minus(trapPose.getTranslation()).getNorm();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//         // return getDistanceToTrap() <= Alignment.TRAP_CLIMB_DISTANCE.get();
//     }
// }
