// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseManager;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class DriveCommands {
  private static final double DEADBAND = 0.05;
  private static Drive drive;
  private static DoubleSupplier xSupplier;
  private static DoubleSupplier ySupplier;
  private static DoubleSupplier omegaSupplier;
  private static BooleanSupplier fastMode;
  private static LoggedDashboardNumber slowDriveMultiplier;
  private static LoggedDashboardNumber slowTurnMultiplier;
  private static PoseManager poseManager;

  public DriveCommands(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fastMode,
      LoggedDashboardNumber slowDriveMultiplier,
      LoggedDashboardNumber slowTurnMultiplier,
      PoseManager poseManager) {
    DriveCommands.drive = drive;
    DriveCommands.xSupplier = xSupplier;
    DriveCommands.ySupplier = ySupplier;
    DriveCommands.omegaSupplier = omegaSupplier;
    DriveCommands.fastMode = fastMode;
    DriveCommands.slowDriveMultiplier = slowDriveMultiplier;
    DriveCommands.slowTurnMultiplier = slowTurnMultiplier;
    DriveCommands.poseManager = poseManager;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive() {
    return Commands.run(
        () -> {
          // Convert to doubles
          double o = omegaSupplier.getAsDouble();

          // Check for slow mode
          if (fastMode.getAsBoolean()) {
            o *= slowTurnMultiplier.get();
          }

          // Apply deadband
          double omega = MathUtil.applyDeadband(o, DEADBAND);

          // Square values
          omega = Math.copySign(omega * omega, omega);

          // Get linear velocity
          Translation2d linearVelocity = getGoalLinearVelocityFromJoysticks();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
                  omega * DriveConstants.MAX_ANGULAR_SPEED,
                  AllianceFlipUtil.shouldFlip()
                      ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                      : poseManager.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using one joystick (controlling linear velocity) with a
   * ProfiledPID for angular velocity.
   */
  public static Command headingDrive(DoubleSupplier desiredAngle) {
    return Commands.run(
        () -> {
          // Get angular velocity
          double omega = getGoalAngularVelocityFromProfiledPID();

          // Get linear velocity
          Translation2d linearVelocity = getGoalLinearVelocityFromJoysticks();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
                  linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
                  omega * DriveConstants.MAX_ANGULAR_SPEED,
                  AllianceFlipUtil.shouldFlip()
                      ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                      : poseManager.getRotation()));
        },
        drive);
  }

  private static Translation2d getGoalLinearVelocityFromJoysticks() {
    // Convert to doubles
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();

    // Check for slow mode
    if (fastMode.getAsBoolean()) {
      double multiplier = slowDriveMultiplier.get();
      x *= multiplier;
      y *= multiplier;
    }

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    return linearVelocity;
  }

  private static double getGoalAngularVelocityFromProfiledPID() {
    return 0; // TODO make getGoalAngularVelocity work
  }
}
