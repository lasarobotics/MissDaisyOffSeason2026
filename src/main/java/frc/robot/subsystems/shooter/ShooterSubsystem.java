// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.HeadHoncho;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

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
        if (DriveSubsystem.getInstance().atGoodShootingPosition()) {
          if ((getInstance().atUnwindAngle() || getInstance().m_isUnwinding)
              && !getInstance().finishedUnwind()) {
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

  private CANcoder m_encoderOne;
  private CANcoder m_encoderTwo;

  private ShooterStates m_requestedState;

  private boolean m_isUnwinding;
  private boolean m_isDriveUnwinding;

  private VelocityDutyCycle m_shooterVelocityDutyCycle;

  private PositionVoltage m_positionRequest;

  public ShooterSubsystem() {
    super(ShooterStates.REST);

    m_positionRequest = new PositionVoltage(Degrees.of(0));

    m_requestedState = ShooterStates.SHOOT;

    m_shooterVelocityDutyCycle = new VelocityDutyCycle(0);

    m_isUnwinding = false;
    m_isDriveUnwinding = false;

    m_flywheelLeaderMotor = new TalonFX(Constants.ShooterConstants.FLYWHEEL_LEADER_CAN_ID);
    m_flywheelFollowerMotor = new TalonFX(Constants.ShooterConstants.FLYWHEEL_FOLLOWER_CAN_ID);
    m_hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_CAN_ID);
    m_turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_CAN_ID);

    m_encoderOne = new CANcoder(Constants.ShooterConstants.ENCODER_ONE_CAN_ID);
    m_encoderTwo = new CANcoder(Constants.ShooterConstants.ENCODER_TWO_CAN_ID);

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

    new Thread(() -> updateTurretPosition()).start();
  }

  public AngularVelocity getDesiredShooterVelocity(Angle hoodAngle, Translation2d target) {
    Distance D = DriveSubsystem.getInstance().getDistance(target);
    Distance dh =
        Meters.of(
            Constants.FieldConstants.HUB_Y_POS
                - Constants.ShooterConstants.SHOOTER_OFFSET_Z.in(Meters));
    double v =
        Math.sqrt(
            (Constants.FieldConstants.GRAVITY_VALUE * Math.pow(D.in(Meters), 2))
                / ((2 * Math.pow(Math.pow(hoodAngle.in(Degrees), 2), 2))
                    * (D.in(Meters) * Math.tan(hoodAngle.in(Degrees)) - dh.in(Meters))));
    return RotationsPerSecond.of(
        v
            / (Math.PI
                * ((Constants.ShooterConstants.FLYWHEEL_SMALL_DIAMETER
                        + Constants.ShooterConstants.FLYWHEEL_LARGE_DIAMETER)
                    / 2)));
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
        MathUtil.clamp(
            Degrees.of(
                    Math.atan(
                        (dh.in(Meters)
                                + Math.sqrt(Math.pow(D.in(Meters), 2) + Math.pow(dh.in(Meters), 2)))
                            / D.in(Meters)))
                .in(Degrees),
            Constants.ShooterConstants.HOOD_MINIMUM_ANGLE.in(Degrees),
            Constants.ShooterConstants.HOOD_MAX_ANGLE.in(Degrees)));
  }

  public Angle getDesiredTurretAngle(Translation2d target) {
    Pose2d robotPose = DriveSubsystem.getInstance().getRobotPose();
    Translation2d robotTranslation = robotPose.getTranslation();
    Distance distanceToTarget = DriveSubsystem.getInstance().getDistance(target);
    double robotRotationDiff =
        Math.acos((target.getX() - robotTranslation.getX() / distanceToTarget.in(Meters)));
    return Degrees.of(robotRotationDiff);
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
                getInstance()
                    .getDesiredShooterVelocity(desiredHoodAngle, getInstance().getShootingTarget())
                    .in(RotationsPerSecond))
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
    double degreesTurretPos = getInstance().m_turretMotor.getPosition().getValue().in(Degrees);
    return (degreesTurretPos * Constants.ShooterConstants.TURRET_THRESHOLD
            >= Constants.ShooterConstants.TURRET_MAX_ANGLE.in(Degrees)
        || degreesTurretPos * Constants.ShooterConstants.TURRET_THRESHOLD
            <= Constants.ShooterConstants.TURRET_MINIMUM_ANGLE.in(Degrees));
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
        getInstance()
            .getDesiredShooterVelocity(desiredHoodAngle, getInstance().getShootingTarget());
    Angle desiredTurretAngle = getInstance().getDesiredTurretAngle(target);

    getInstance().setShooterVelocity(desiredShooterVelocity);
    getInstance().setHoodAngle(desiredHoodAngle);
    getInstance().setTurretAngle(desiredTurretAngle);
  }

  public void unwindTurret() {
    getInstance()
        .m_turretMotor
        .setControl(getInstance().m_positionRequest.withPosition(Degrees.of(0)));
  }

  public boolean finishedUnwind() {
    if (getInstance().m_turretMotor.getPosition()
        == Constants.ShooterConstants.TURRET_UNWIND_ANGLE) {
      getInstance().m_isUnwinding = false;
      return true;
    }
    return false;
  }

  public Translation2d getShootingTarget() {
    Translation2d robotTranslation = DriveSubsystem.getInstance().getRobotPose().getTranslation();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (robotTranslation.getX() < Constants.FieldConstants.BLUE_ZONE_X) {
        return Constants.FieldConstants.BLUE_HUB_COORDINATES;
      }
      if (robotTranslation.getX() > Constants.FieldConstants.BLUE_ZONE_X
          && robotTranslation.getX() < Constants.FieldConstants.RED_ZONE_X) {
        if (robotTranslation.getY() < Constants.FieldConstants.HALF_FIELD_Y_POS) {
          return Constants.FieldConstants.BLUE_AZ_PASS_LEFT;
        } else {
          return Constants.FieldConstants.BLUE_AZ_PASS_RIGHT;
        }
      }
      if (robotTranslation.getX() > Constants.FieldConstants.RED_ZONE_X) {
        if (robotTranslation.getY() < Constants.FieldConstants.HALF_FIELD_Y_POS) {
          return Constants.FieldConstants.BLUE_NZ_PASS_LEFT;
        } else {
          return Constants.FieldConstants.BLUE_NZ_PASS_RIGHT;
        }
      }
    } else {
      if (robotTranslation.getX() > Constants.FieldConstants.RED_ZONE_X) {
        return Constants.FieldConstants.RED_HUB_COORDINATES;
      }
      if (robotTranslation.getX() > Constants.FieldConstants.BLUE_ZONE_X
          && robotTranslation.getX() < Constants.FieldConstants.RED_ZONE_X) {
        if (robotTranslation.getY() < Constants.FieldConstants.HALF_FIELD_Y_POS) {
          return Constants.FieldConstants.RED_AZ_PASS_LEFT;
        } else {
          return Constants.FieldConstants.RED_AZ_PASS_RIGHT;
        }
      }
      if (robotTranslation.getX() < Constants.FieldConstants.BLUE_ZONE_X) {
        if (robotTranslation.getY() < Constants.FieldConstants.HALF_FIELD_Y_POS) {
          return Constants.FieldConstants.RED_NZ_PASS_LEFT;
        } else {
          return Constants.FieldConstants.RED_NZ_PASS_RIGHT;
        }
      }
    }

    return Constants.FieldConstants.FIELD_CENTER;
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
        && DriveSubsystem.getInstance().atGoodShootingPosition()
        && getInstance().atGoodTurretAngle(getInstance().getDesiredHoodAngle(getShootingTarget())));
  }

  public void updateTurretPosition() {
    StatusSignal<Angle> encoderASignal = m_encoderOne.getPosition();
    StatusSignal<Angle> encoderBSignal = m_encoderTwo.getPosition();
    BaseStatusSignal.refreshAll(encoderASignal, encoderBSignal);
    BaseStatusSignal.waitForAll(0.1, encoderASignal, encoderBSignal);
    double encoderAPosition = encoderASignal.getValue().in(Degrees);
    double encoderBPosition = encoderBSignal.getValue().in(Degrees);

    double[] encoderOnePossible = new double[Constants.ShooterConstants.ENCODER_TEETH_ONE];
    double[] encoderTwoPossible = new double[Constants.ShooterConstants.ENCODER_TEETH_TWO];

    // for encoder one
    for (int i = 0; i < Constants.ShooterConstants.ENCODER_TEETH_TWO; i++) {
      encoderOnePossible[i] =
          (i + (encoderAPosition / 360))
              * ((double) Constants.ShooterConstants.ENCODER_TEETH_ONE
                  / Constants.ShooterConstants.TURRET_GEAR_TEETH);
    }
    // for encoder two
    for (int i = 0; i < Constants.ShooterConstants.ENCODER_TEETH_ONE; i++) {
      encoderTwoPossible[i] =
          (i + (encoderBPosition / 360))
              * ((double) Constants.ShooterConstants.ENCODER_TEETH_TWO
                  / Constants.ShooterConstants.TURRET_GEAR_TEETH);
    }

    double matchingValue = 0;
    outerLoop:
    for (double eOnePossible : encoderOnePossible) {
      for (double eTwoPossible : encoderTwoPossible) {
        if (Math.abs(eTwoPossible - eOnePossible) < Constants.ShooterConstants.CRT_THRESHOLD) {
          matchingValue = (eOnePossible + eTwoPossible) / 2;
          break outerLoop;
        }

        if (eTwoPossible > eOnePossible) {
          break;
        }
      }
    }
    m_turretMotor.setPosition(matchingValue);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ShooterSubsystem/State", getState().toString());
    Logger.recordOutput("ShooterSubsystem/HoodAngle", m_hoodMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "ShooterSubsystem/DesiredHoodAngle", getDesiredHoodAngle(getShootingTarget()));
    Logger.recordOutput(
        "ShooterSubsystem/DesiredShooterVelocity",
        getDesiredShooterVelocity(getDesiredHoodAngle(getShootingTarget()), getShootingTarget()));
    Logger.recordOutput(
        "ShooterSubsystem/DesiredTurretAngle", getDesiredTurretAngle(getShootingTarget()));
    Logger.recordOutput("ShooterSubsystem/ShooterTarget", getShootingTarget());
    Logger.recordOutput(
        "ShooterSubsystem/ShooterVelocity", m_flywheelLeaderMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "ShooterSubsystem/TurretAngle", m_turretMotor.getPosition().getValueAsDouble());
  }
}
