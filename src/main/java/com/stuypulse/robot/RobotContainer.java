package com.stuypulse.robot;

import com.ctre.phoenix6.Utils;
import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.arm.ArmToAmp;
import com.stuypulse.robot.commands.arm.ArmToClimbing;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToLobFerry;
import com.stuypulse.robot.commands.arm.ArmToLobFerryManual;
import com.stuypulse.robot.commands.arm.ArmToLowFerry;
import com.stuypulse.robot.commands.arm.ArmToLowFerryManual;
import com.stuypulse.robot.commands.arm.ArmToPreClimb;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.CenterMobilityWithWait;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.Mobility;
import com.stuypulse.robot.commands.auton.RerouteTest;
import com.stuypulse.robot.commands.auton.ADEF.FivePieceADEF;
import com.stuypulse.robot.commands.auton.BCA.FourPieceBCA;
import com.stuypulse.robot.commands.auton.BCA.RightAngleFourPieceBCA;
import com.stuypulse.robot.commands.auton.BF_Series.FivePieceBFGH;
import com.stuypulse.robot.commands.auton.BF_Series.SixPieceBDEFA;
import com.stuypulse.robot.commands.auton.BF_Series.SixPieceBFCAD;
import com.stuypulse.robot.commands.auton.HGF.ThreePieceGH;
import com.stuypulse.robot.commands.auton.HGF.ThreePieceHG;
import com.stuypulse.robot.commands.auton.HGF.FourPieceHGF;
import com.stuypulse.robot.commands.auton.HGF.ReroutableFourPieceHGF;
import com.stuypulse.robot.commands.auton.SideAutons.OnePieceAmpSide;
import com.stuypulse.robot.commands.auton.SideAutons.OnePieceSourceSide;
import com.stuypulse.robot.commands.auton.tests.StraightLine;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.leds.LEDDefaultMode;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterFeederAcquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederDeacquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveRobotRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedManualFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedSpeaker;
import com.stuypulse.robot.commands.vision.VisionChangeWhiteList;
import com.stuypulse.robot.commands.vision.VisionDisable;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.Telemetry;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final AprilTagVision vision = AprilTagVision.getInstance();
    
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Arm arm = Arm.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();

    public final LEDController leds = LEDController.getInstance();

    private final Telemetry logger = new Telemetry();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        if (Utils.isSimulation()) {
            swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        }
        swerve.registerTelemetry(logger::telemeterize);

        LiveWindow.disableAllTelemetry();

        new Trigger(() -> Intake.getInstance().getState() == Intake.State.ACQUIRING && Intake.getInstance().hasNote()
                    || ((driver.getLeftTriggerPressed() || driver.getRightTriggerPressed()) && (Intake.getInstance().hasNote() || Shooter.getInstance().hasNote())))
            .onTrue(new BuzzController(driver, 1, 1));

        // new VisionDisable();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        leds.setDefaultCommand(new LEDDefaultMode());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureOperatorBindings();
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        driver.getDPadRight().onTrue(new SwerveDriveSeedFieldRelative());

        driver.getDPadUp()
            .onTrue(new ArmToPreClimb())
            .onTrue(new ShooterFeederStop())
            .onTrue(new IntakeStop());

        driver.getDPadDown().onTrue(new ArmToClimbing());

        // intake with either trigger and when driving
        new Trigger(() -> (driver.getRightTriggerPressed() 
                        || driver.getLeftTriggerPressed() 
                        || (driver.getLeftStick().distance() > Settings.Driver.Drive.DEADBAND.get() + 0.1 
                            && !Intake.getInstance().hasNote()
                            && !Shooter.getInstance().hasNote())))
            .onTrue(new IntakeSetAcquire())
            .onFalse(new IntakeStop());
        
        driver.getRightTriggerButton().onTrue(new ArmToFeed());
        
        // drive robot relative
        driver.getLeftTriggerButton()
            .onTrue(new ArmToFeed())
            .whileTrue(new SwerveDriveDriveRobotRelative(driver))
            .whileTrue(new LEDSet(LEDInstructions.ROBOT_RELATIVE_INTAKING));
        
        // deacquire
        driver.getDPadLeft()
            .onTrue(new IntakeDeacquire())
            .onTrue(new ShooterFeederDeacquire())
            .whileTrue(new LEDSet(LEDInstructions.DEACQUIRING))
            .onFalse(new IntakeStop())
            .onFalse(new ShooterFeederStop());
        
        // speaker align and score 
        // score amp
        driver.getRightBumper()
            .onTrue(new ConditionalCommand(
                new VisionChangeWhiteList(7, 8), 
                new VisionChangeWhiteList(3, 4), 
                () -> Robot.isBlue()))
            .whileTrue(new ConditionalCommand(
                new SwerveDriveDrive(driver)
                    .alongWith(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                        .andThen(new ShooterFeederDeacquire().alongWith(new LEDSet(LEDInstructions.AMP_SCORE)))),
                new SwerveDriveDriveAlignedSpeaker(driver)
                    .alongWith(new ArmToSpeaker()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET))
                                .alongWith(new WaitUntilCommand(() -> swerve.isAlignedToSpeaker())))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.SPEAKER_ALIGN)),
                () -> Arm.getInstance().getState() == Arm.State.AMP))
            .onFalse(new ShooterFeederStop())
            .onFalse(new ArmToFeed().onlyIf(() -> arm.getState() == Arm.State.SPEAKER))
            .onFalse(new VisionReloadWhiteList());

        // lob ferry align and shoot
        driver.getLeftStickButton()
            .whileTrue(new SwerveDriveDriveAlignedFerry(driver)
                    .alongWith(new ArmToLobFerry()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToFerry()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOB_FERRY_ALIGN))
            )
            .onFalse(new ShooterFeederStop())
            .onFalse(new ArmToFeed());


        // low ferry align and shoot
        driver.getRightStickButton()
            .whileTrue(new SwerveDriveDriveAlignedFerry(driver)
                    .alongWith(new ArmToLowFerry()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToFerry()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOW_FERRY_ALIGN))
            )
            .onFalse(new ShooterFeederStop())
            .onFalse(new ArmToFeed());

        // arm to amp
        driver.getLeftBumper().onTrue(new ArmToAmp());

        // manual speaker at subwoofer
        driver.getRightMenuButton()
            .whileTrue(new ArmToSubwooferShot()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new ShooterFeederShoot())
                        )
            .whileTrue(new LEDSet(LEDInstructions.SPEAKER_MANUAL))
            .onFalse(new ShooterFeederStop())
            .onFalse(new ArmToFeed());
        
        // manual lob ferry
        driver.getTopButton()
            .whileTrue(new SwerveDriveDriveAlignedManualFerry(driver)
                    .alongWith(new ArmToLobFerryManual()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToManualFerry()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOB_FERRY_ALIGN_MANUAL))
            )
            .onFalse(new ShooterFeederStop())
            .onFalse(new ArmToFeed());

        // manual low ferry
        driver.getLeftButton()
            .whileTrue(new SwerveDriveDriveAlignedManualFerry(driver)
                    .alongWith(new ArmToLowFerryManual()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToManualFerry()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOW_FERRY_ALIGN_MANUAL))
            )
            .onFalse(new ShooterFeederStop())
            .onFalse(new ArmToFeed());
        
        // human player attention button
        // driver.getRightButton().whileTrue(new LEDSet(LEDInstructions.ATTENTION));

        // "special deacquire"
        driver.getRightButton()
            .onTrue(new IntakeDeacquire())
            .onTrue(new ShooterFeederAcquire())
            .onFalse(new IntakeStop())
            .onFalse(new ShooterFeederStop());

        // driver.getRightButton()
        //     .whileTrue(new SwerveDriveToPose(() -> Field.getAllianceSpeakerPose().plus(new Transform2d(3.75, 0, new Rotation2d()))));
    }

    private void configureOperatorBindings() {
        operator.getRightTriggerButton().whileTrue(new LEDSet(LEDInstructions.RAINBOW));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());
        
        // Mobility
        AutonConfig MOBILITY_BLUE = new AutonConfig("Mobility", Mobility::new, "Mobility");
        AutonConfig MOBILITY_RED = new AutonConfig("Mobility", Mobility::new, "Mobility");

        // BCA
        AutonConfig BCA_BLUE = new AutonConfig("4 BCA", FourPieceBCA::new,
        "Blue Center to B", "Blue B to Center","Blue 90 Deg B Shoot to C", "Blue C to Shoot Before A", "Blue Center to A");
        AutonConfig BCA_RED = new AutonConfig("4 BCA", FourPieceBCA::new,
        "Red Center to B", "Red B to Center", "Red 90 Deg B Shoot to C", "Red C to Shoot Before A", "Red Center to A");

       // HGF
        AutonConfig HGF_BLUE = new AutonConfig("4 HGF", FourPieceHGF::new,
        "Blue Source to H", "Blue H to Shoot", "Blue H Shoot to G", "Blue G to Shoot", "Blue G Shoot to F", "Blue F to Shoot");
        AutonConfig HGF_RED = new AutonConfig("4 HGF", FourPieceHGF::new,
        "Red Source to H", "Red H to Shoot", "Red H Shoot to G", "Red G to Shoot", "Red G Shoot to F", "Red F to Shoot");

        // GH
        AutonConfig GH_BLUE = new AutonConfig("3 GH", ThreePieceGH::new,
        "Blue Source to G", "Blue G to Shoot", "Blue G Shoot to H", "Blue H to Shoot");
        AutonConfig GH_RED = new AutonConfig("3 GH", ThreePieceGH::new,
        "Red Source to G", "Red G to Shoot", "Red G Shoot to H", "Red H to Shoot");

        // HG
        AutonConfig HG_BLUE = new AutonConfig("3 HG", ThreePieceHG::new,
        "Blue Source to H", "Blue H to Shoot", "Blue H Shoot to G", "Blue G to Shoot");
        AutonConfig HG_RED = new AutonConfig("3 HG", ThreePieceHG::new,
        "Red Source to H", "Red H to Shoot", "Red H Shoot to G", "Red G to Shoot");

        // Reroutable HGF
        //AutonConfig ReroutableHGF_BLUE = new AutonConfig("4 HGF Reroute", ReroutableFourPieceHGF::new, 
        //    "Blue Source to H", "Blue H to Shoot", "Blue H Shoot to G", "Blue G to Shoot", "Blue G Shoot to F", "Blue F to Shoot", "Blue H to G Reroute", "Blue G to F Reroute");
        //AutonConfig ReroutableHGF_RED = new AutonConfig("4 HGF Reroute", ReroutableFourPieceHGF::new, 
        //"Red Source to H", "Red H to Shoot", "Red H Shoot to G", "Red G to Shoot", "Red G Shoot to F", "Red F to Shoot", "Red H to G Reroute", "Red G to F Reroute");
        
        // ADEF
        AutonConfig ADEF_BLUE = new AutonConfig("5 ADEF", FivePieceADEF::new,
        "Blue Amp to A", "Blue A to D", "Blue D to Shoot", "Blue D Shoot to E", "Blue E to Shoot", "Blue E Shoot to F", "Blue F to Shoot");
        AutonConfig ADEF_RED = new AutonConfig("5 ADEF", FivePieceADEF::new,
        "Red Amp to A", "Red A to D", "Red D to Shoot", "Red D Shoot to E", "Red E to Shoot", "Red E Shoot to F", "Red F to Shoot");

        // BDEFA
        AutonConfig BDEFA_BLUE = new AutonConfig("6 BDEFA", SixPieceBDEFA::new,
        "Blue Center to B", "Blue B to D", "Blue D to Shoot", "Blue D Shoot to E", "Blue E to Shoot", "Blue E Shoot to F", "Blue F to Shoot", "Blue F Shoot to A", "Blue Center to A");
        AutonConfig BDEFA_RED = new AutonConfig("6 BDEFA", SixPieceBDEFA::new,
        "Red Center to B", "Red B to D", "Red D to Shoot", "Red D Shoot to E", "Red E to Shoot", "Red E Shoot to F", "Red F to Shoot", "Red F Shoot to A", "Red Center to A");

        // BFCAD
        AutonConfig BFCAD_BLUE = new AutonConfig("6 BFCAD", SixPieceBFCAD::new,
        "Blue Center to B", "Blue B to F", "Blue F to Close C Shoot", "Blue FC Shoot to C", "Blue C to Shoot Before A", "Blue Center to A", "Blue A to Center", "Blue A Shoot to D", "Blue D to Shoot");
        AutonConfig BFCAD_RED = new AutonConfig("6 BFCAD", SixPieceBFCAD::new,
        "Red Center to B", "Red B to F", "Red F to Close C Shoot", "Red FC Shoot to C", "Red C to Shoot Before A", "Red Center to A", "Red A to Center", "Red A Shoot to D", "Red D to Shoot");

        // BFGH
        AutonConfig BFGH_BLUE = new AutonConfig("5 BFGH", FivePieceBFGH:: new,
        "Blue Center to B", "Blue B to F", "Blue F to Shoot", "Blue F Shoot to G", "Blue G to F Shoot", "Blue GF Shoot to H", "Blue H to F Shoot");
        AutonConfig BFGH_RED = new AutonConfig("5 BFGH", FivePieceBFGH:: new,
        "Red Center to B", "Red B to F", "Red F to Shoot", "Red F Shoot to G", "Red G to F Shoot", "Red GF Shoot to H", "Red H to F Shoot");
        
        // Reroute Test
        //AutonConfig Reroute_Test_Blue = new AutonConfig("Reroute Test", RerouteTest::new,
        //"Blue Center to B", "Blue B to Center", "B to A Reroute Test", "Blue A to Center");
        //AutonConfig Reroute_Test_Red = new AutonConfig("Reroute Test", RerouteTest::new,
        //"Red Center to B", "Red B to Center", "B to A Reroute Test", "Red A to Center");

        // New BCA
        AutonConfig New_BCA_Blue = new AutonConfig("New BCA", RightAngleFourPieceBCA::new,
        "Blue Center to B", "Blue B to Center", "Blue 90 Deg B Shoot to C", "Blue C to Shoot Before A", "Blue Center to A", "Blue A to Center");
        AutonConfig New_BCA_Red = new AutonConfig("New BCA", RightAngleFourPieceBCA::new,
        "Red Center to B", "Red B to Center", "Red 90 Deg B Shoot to C", "Red C to Shoot Before A", "Red Center to A", "Red A to Center");

        AutonConfig One_Piece_Mobility_Amp_Side_Blue = new AutonConfig("One Piece Amp Side", OnePieceAmpSide::new, 
            "Blue Amp Side Mobility");

        // Straight Line
        AutonConfig Straight_Line = new AutonConfig("Straight Line Test", StraightLine::new,
        "Straight Line");

        One_Piece_Mobility_Amp_Side_Blue.registerBlue(autonChooser);

        Straight_Line.registerBlue(autonChooser);

        MOBILITY_BLUE.registerBlue(autonChooser);
        MOBILITY_RED.registerRed(autonChooser);

        BCA_BLUE.registerDefaultBlue(autonChooser);
        BCA_RED.registerDefaultRed(autonChooser);

        New_BCA_Blue.registerDefaultBlue(autonChooser);
        New_BCA_Red.registerDefaultRed(autonChooser);

        BDEFA_BLUE.registerBlue(autonChooser);
        BDEFA_RED.registerRed(autonChooser);

        BFCAD_BLUE.registerBlue(autonChooser);
        BFCAD_RED.registerRed(autonChooser);

        BFGH_BLUE.registerBlue(autonChooser);
        BFGH_RED.registerRed(autonChooser);

        HGF_BLUE.registerBlue(autonChooser);
        HGF_RED.registerRed(autonChooser);

        HG_BLUE.registerBlue(autonChooser);
        HG_RED.registerRed(autonChooser);

        GH_BLUE.registerBlue(autonChooser);
        GH_RED.registerRed(autonChooser);

        //ReroutableHGF_BLUE.registerBlue(autonChooser);
        //ReroutableHGF_RED.registerRed(autonChooser);

        ADEF_BLUE.registerBlue(autonChooser);
        ADEF_RED.registerRed(autonChooser);

        //Reroute_Test_Blue.registerBlue(autonChooser);
        //Reroute_Test_Red.registerRed(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static String getAutonomousCommandNameStatic() {
        if (autonChooser.getSelected() == null) {
            return "Do Nothing";
        }

        return autonChooser.getSelected().getName();
    }
}