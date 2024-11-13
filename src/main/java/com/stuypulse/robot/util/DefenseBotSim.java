package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DefenseBotSim extends SubsystemBase {

  private static DefenseBotSim instance;

  static {
    instance = new DefenseBotSim(2);
  }

  private final Gamepad controller;
  private Pose2d pose;
  private final FieldObject2d robot; 

  private final SlewRateLimiter xStickLimiter =
      new SlewRateLimiter(4);
  private final SlewRateLimiter yStickLimiter =
      new SlewRateLimiter(4);
  private final SlewRateLimiter rotStickLimiter =
      new SlewRateLimiter(4);

  double xVelocity;
  double yVelocity;

  public DefenseBotSim(int controllerPort) {
    this.controller = new AutoGamepad(controllerPort);
    this.pose = new Pose2d(3, 3, new Rotation2d());
    this.robot = SwerveDrive.getInstance().getField().getObject("Defense Bot");

    xStickLimiter.reset(controller.getLeftY());
    yStickLimiter.reset(controller.getLeftX());
    rotStickLimiter.reset(-controller.getRightX());
    xVelocity = 0;
    yVelocity = 0;
  }

  public static DefenseBotSim getInstance() {
    return instance;
  }

  public double getVelocityX() {
    return xVelocity;
  }

  public double getVelocityY() {
    return yVelocity;
  }

  public Pose2d getPose() {
    return robot.getPose();
  }

  @Override
  public void periodic() {
    xVelocity = xStickLimiter.calculate(controller.getLeftY()) * 3.0;
    yVelocity = yStickLimiter.calculate(controller.getLeftX()) * 3.0;
    double rot = rotStickLimiter.calculate(-controller.getRightX()) * 360;

    Translation2d deltaPos = new Translation2d(xVelocity, yVelocity).times(Settings.DT);
    Rotation2d deltaRot = Rotation2d.fromDegrees(rot).times(Settings.DT);

    this.pose = new Pose2d(pose.getTranslation().plus(deltaPos), pose.getRotation().plus(deltaRot));

    this.robot.setPose(pose);
  }
}
