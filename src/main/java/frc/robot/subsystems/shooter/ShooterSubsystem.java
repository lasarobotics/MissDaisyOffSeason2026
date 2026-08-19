// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.HeadHoncho;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShooterSubsystem extends StateMachine {

  public enum ShooterStates implements SystemState {
    REST {
      @Override
      public void execute() {
        getInstance()
            .setTurretAngle(getInstance().getDesiredTurretAngle(getInstance().getShootingTarget()));
        getInstance()
            .setHoodAngle(getInstance().getDesiredHoodAngle(getInstance().getShootingTarget()));
        getInstance().setShooterVelocity(Constants.ShooterConstants.FLYWHEEL_REST_SPEED);
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    SHOOT {
      @Override
      public void execute() {
        if (getInstance().atGoodShootingPosition()) {
          if ((getInstance().atUnwindAngle() || getInstance().m_isUnwinding)
              && getInstance().updateCurrentTurretPos() > 0) {
            getInstance().m_isUnwinding = true;
            if (DriveSubsystem.isCommandedMoving()) {
              getInstance().m_isDriveUnwinding = true;
              HeadHoncho.getInstance().requestDriveUnwind();
            } else {
              getInstance().m_isDriveUnwinding = false;
              HeadHoncho.getInstance().driveUnwindEnded();
              getInstance().unwindTurret();
            }
          } else {
            getInstance().m_isUnwinding = false;
            HeadHoncho.getInstance().driveUnwindEnded();
            getInstance().shoot(getInstance().getShootingTarget());
          }
        } else {
          getInstance().unwindTurret();
        }
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },
  }

  private static ShooterSubsystem s_shooterInstance;

  private TalonFX m_flywheelLeaderMotor;
  private TalonFX m_flywheelFollowerMotor;
  private TalonFX m_hoodMotor;
  private TalonFX m_turretMotor;

  private ShooterStates m_requestedState;

  private boolean m_isUnwinding;
  private boolean m_isDriveUnwinding;

  private double m_currentTurretPosition;

  private VelocityDutyCycle m_shooterVelocityDutyCycle;

  private PositionVoltage m_positionRequest;

  public ShooterSubsystem() {
    super(ShooterStates.REST);

    m_positionRequest = new PositionVoltage(Degrees.of(0));

    m_requestedState = ShooterStates.SHOOT;

    m_isUnwinding = false;
    m_isDriveUnwinding = false;

    m_flywheelLeaderMotor = new TalonFX(Constants.ShooterConstants.FLYWHEEL_LEADER_CAN_ID);
    m_flywheelFollowerMotor = new TalonFX(Constants.ShooterConstants.FLYWHEEL_FOLLOWER_CAN_ID);
    m_hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_CAN_ID);
    m_turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_CAN_ID);

    m_currentTurretPosition = m_turretMotor.getPosition().getValueAsDouble();

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.withKP(0).withKI(0).withKD(0);
    m_flywheelLeaderMotor.getConfigurator().apply(flywheelConfig);
    m_flywheelFollowerMotor.getConfigurator().apply(flywheelConfig);

    m_flywheelFollowerMotor.setControl(
        new Follower(m_flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.withKP(0).withKI(0).withKD(0);
    m_hoodMotor.getConfigurator().apply(hoodConfig);

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.Slot0.withKP(0).withKI(0).withKD(0);
    m_turretMotor.getConfigurator().apply(turretConfig);
  }

  public AngularVelocity getDesiredShooterVelocity(Angle hoodAngle) {
    return RotationsPerSecond.of(0);
  }

  public Angle getDesiredHoodAngle(Translation2d target) {
    if (DriveSubsystem.getInstance().isUnderTrench()) {
      return Degrees.of(Constants.ShooterConstants.HOOD_MINIMUM_ANGLE.in(Degrees));
    }
    Distance D = DriveSubsystem.getInstance().getDistance(target);
    Distance dh =
        Meters.of(
            Constants.FieldConstants.HUB_Y_POS
                - Constants.ShooterConstants.SHOOTER_OFFSET_Z.in(Meters));
    return Degrees.of(
        Math.atan(
            (dh.in(Meters) + Math.sqrt(Math.pow(D.in(Meters), 2) + Math.pow(dh.in(Meters), 2)))
                / D.in(Meters)));
  }

  public Angle getDesiredTurretAngle(Translation2d target) {
    return Degrees.of(0);
  }

  public double getFlightTime(Translation2d target, Angle desiredHoodAngle) {
    Distance D = DriveSubsystem.getInstance().getDistance(target);
    Distance dh =
        Meters.of(
            Constants.FieldConstants.HUB_Y_POS
                - Constants.ShooterConstants.SHOOTER_OFFSET_Z.in(Meters));
    return Math.sqrt(
        ((2 * D.in(Meters) * Math.tan(desiredHoodAngle.in(Degrees) - dh.in(Meters))))
            / Constants.FieldConstants.GRAVITY_VALUE);
  }

  public double getVelocityX(Translation2d target) {
    return DriveSubsystem.getInstance().getDistance(target).in(Meters)
        / getFlightTime(target, getInstance().getDesiredHoodAngle(target));
  }

  public double getVelocityY(Translation2d target) {
    return getVelocityX(target) * Math.tan(getFlightTime(target, getDesiredHoodAngle(target)));
  }

  public double getVelocity(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public AngularVelocity getFuelVelocity(double velocityX, double velocityY) {
    return RotationsPerSecond.of(0);
  }

  public boolean atGoodShootingPosition() {
    return true;
  }

  public boolean atGoodHoodAngle(Angle desiredHoodAngle) {
    return (Math.abs(desiredHoodAngle.in(Degrees))
            < Math.abs(
                getInstance().m_hoodMotor.getPosition().getValue().in(Degrees)
                    * Constants.ShooterConstants.HOOD_THRESHOLD))
        ? true
        : false;
  }

  public boolean atGoodShooterVelocity(Angle desiredHoodAngle) {
    return (Math.abs(
                getInstance().getDesiredShooterVelocity(desiredHoodAngle).in(RotationsPerSecond))
            < Math.abs(
                getInstance().m_flywheelLeaderMotor.getVelocity().getValueAsDouble()
                    * Constants.ShooterConstants.HOOD_THRESHOLD))
        ? true
        : false;
  }

  public boolean atGoodTurretAngle(Angle desiredTurretAngle) {
    return (Math.abs(getInstance().m_turretMotor.getPosition().getValue().in(Degrees))
            < Math.abs(
                getInstance().m_turretMotor.getPosition().getValue().in(Degrees)
                    * Constants.ShooterConstants.TURRET_THRESHOLD))
        ? true
        : false;
  }

  public boolean atUnwindAngle() {
    return getInstance().m_turretMotor.getPosition().getValue().in(Degrees)
        >= Constants.ShooterConstants.TURRET_MAX_ANGLE.in(Degrees);
  }

  public double updateCurrentTurretPos() {
    double lastTurretPosition = getInstance().m_currentTurretPosition;
    getInstance().m_currentTurretPosition =
        getInstance().m_turretMotor.getPosition().getValueAsDouble();
    return getInstance().m_currentTurretPosition - lastTurretPosition;
  }

  public void setShooterVelocity(AngularVelocity shooterVelocity) {
    getInstance()
        .m_flywheelLeaderMotor
        .setControl(getInstance().m_shooterVelocityDutyCycle.withVelocity(shooterVelocity));
  }

  public void setHoodAngle(Angle hoodAngle) {
    getInstance().m_hoodMotor.setControl(getInstance().m_positionRequest.withPosition(hoodAngle));
  }

  public void setTurretAngle(Angle turretAngle) {
    getInstance()
        .m_turretMotor
        .setControl(getInstance().m_positionRequest.withPosition(turretAngle));
  }

  public void shoot(Translation2d target) {
    Angle desiredHoodAngle = getInstance().getDesiredHoodAngle(target);
    AngularVelocity desiredShooterVelocity =
        getInstance().getDesiredShooterVelocity(desiredHoodAngle);
    Angle desiredTurretAngle = getInstance().getDesiredTurretAngle(target);

    getInstance().setShooterVelocity(desiredShooterVelocity);
    getInstance().setHoodAngle(desiredHoodAngle);
    getInstance().setTurretAngle(desiredTurretAngle);
  }

  public void unwindTurret() {}

  public Translation2d getShootingTarget() {
    return new Translation2d();
  }

  public boolean getIsDriveUnwinding() {
    return getInstance().m_isDriveUnwinding;
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }
    return s_shooterInstance;
  }

  public void setState(ShooterStates state) {
    getInstance().m_requestedState = state;
  }

  public boolean isShooterReady() {
    return (getInstance().atGoodHoodAngle(getInstance().getDesiredHoodAngle(getShootingTarget()))
        && getInstance()
            .atGoodShooterVelocity(getInstance().getDesiredHoodAngle(getShootingTarget()))
        && getInstance().atGoodShootingPosition()
        && getInstance().atGoodTurretAngle(getInstance().getDesiredHoodAngle(getShootingTarget())));
  }

  @Override
  public void periodic() {
    getInstance().updateCurrentTurretPos();
  }
}
