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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOMixed implements ModuleIO {
  private final TalonFX driveTalon;
  private final CANSparkMax turnSparkMax;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final double DRIVE_GEAR_RATIO = 6.12244897959;
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isDriveMotorInverted;
  private final boolean isTurnMotorInverted;
  private final boolean isCancoderInverted;
  private final double absoluteEncoderOffseRot; // TODO tune

  private final LoggedTunableNumber drive_kS =
      new LoggedTunableNumber("drive_kS", 0.05); // Add 0.05 V output to overcome static friction
  private final LoggedTunableNumber drive_kV =
      new LoggedTunableNumber(
          "drive_kV", 0.0); // A velocity target of 1 rps results in 0.02 V output
  private final LoggedTunableNumber drive_kP =
      new LoggedTunableNumber("drive_kP", 0.0); // An error of 1 rps results in 0.11 V output
  private final LoggedTunableNumber drive_kD =
      new LoggedTunableNumber("drive_kD", 0.0); // no output for error derivative

  public ModuleIOMixed(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(0);
        turnSparkMax = new CANSparkMax(0, MotorType.kBrushless);
        cancoder = new CANcoder(0);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      case 1:
        driveTalon = new TalonFX(1);
        turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        cancoder = new CANcoder(1);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      case 2:
        driveTalon = new TalonFX(2);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        cancoder = new CANcoder(2);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      case 3:
        driveTalon = new TalonFX(3);
        turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        cancoder = new CANcoder(3);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.setSecondaryCurrentLimit(80, 1);
    turnSparkMax.enableVoltageCompensation(12.0);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    turnSparkMax.setCANTimeout(0);
    turnSparkMax.burnFlash();

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    var config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection =
        isCancoderInverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.MagnetOffset = absoluteEncoderOffseRot;
    cancoder.getConfigurator().apply(config);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);
    driveTalon.optimizeBusUtilization();

    updateTunables();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent)
            .isOK();
    inputs.cancoderConnected = BaseStatusSignal.refreshAll(turnAbsolutePosition).isOK();

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();

    updateTunables();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isDriveMotorInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          Slot0Configs slot0Configs = new Slot0Configs();
          slot0Configs.kS = drive_kS.get();
          slot0Configs.kV = drive_kV.get();
          slot0Configs.kP = drive_kP.get();
          slot0Configs.kD = drive_kD.get();
          driveTalon.getConfigurator().apply(slot0Configs);
        },
        drive_kS,
        drive_kV,
        drive_kP,
        drive_kD);
  }
}
