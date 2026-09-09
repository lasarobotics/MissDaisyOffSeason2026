// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
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
        getInstance().setTurretAngle(getInstance().shot.turretAngle);
        getInstance().setHoodAngle(getInstance().shot.hoodAngle());
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

            getInstance().shoot(getInstance().target, getInstance().hub);
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

  private Translation2d target;
  private boolean hub;
  private ShotSolution shot;

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
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 200;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 120;
    flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;

    m_flywheelLeaderMotor.getConfigurator().apply(flywheelConfig);
    m_flywheelFollowerMotor.getConfigurator().apply(flywheelConfig);

    m_flywheelFollowerMotor.setControl(
        new Follower(m_flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.withKP(0).withKI(0).withKD(0);
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 200;
    hoodConfig.CurrentLimits.StatorCurrentLimit = 120;
    hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    hoodConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;

    m_hoodMotor.getConfigurator().apply(hoodConfig);

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.Slot0.withKP(0).withKI(0).withKD(0);
    turretConfig.CurrentLimits.SupplyCurrentLimit = 200;
    turretConfig.CurrentLimits.StatorCurrentLimit = 120;
    turretConfig.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    turretConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    turretConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;

    m_turretMotor.getConfigurator().apply(turretConfig);

    new Thread(() -> updateTurretPosition()).start();
  }

  public record ShotSolution(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity) {}

  public LinearVelocity getTurretVelocityX() {
    ChassisSpeeds speeds = DriveSubsystem.getSpeeds();

    double robotHeading = DriveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

    Translation2d turretPos =
        new Translation2d(
            Constants.ShooterConstants.SHOOTER_OFFSET_X.in(Meters),
            Constants.ShooterConstants.SHOOTER_OFFSET_Y.in(Meters));

    double turretVelocityRobotX =
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * turretPos.getY();

    double turretVelocityRobotY =
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * turretPos.getX();

    return MetersPerSecond.of(
        turretVelocityRobotX * Math.cos(robotHeading)
            - turretVelocityRobotY * Math.sin(robotHeading));
  }

  public LinearVelocity getTurretVelocityY() {
    ChassisSpeeds speeds = DriveSubsystem.getSpeeds();

    double robotHeading = DriveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

    Translation2d turretPos =
        new Translation2d(
            Constants.ShooterConstants.SHOOTER_OFFSET_X.in(Meters),
            Constants.ShooterConstants.SHOOTER_OFFSET_Y.in(Meters));

    double turretVelocityRobotX =
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * turretPos.getY();

    double turretVelocityRobotY =
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * turretPos.getX();

    return MetersPerSecond.of(
        turretVelocityRobotX * Math.sin(robotHeading)
            + turretVelocityRobotY * Math.cos(robotHeading));
  }

  public static Angle hoodToElevation(Angle hoodAngle) {
    return Constants.ShooterConstants.HOOD_ELEVATION_OFFSET.plus(
        hoodAngle.times(Constants.ShooterConstants.HOOD_ELEVATION_SCALAR));
  }

  public LinearVelocity getFuelVelocityX(
      Angle hoodAngle, Angle turretAngle, AngularVelocity launchSpeed) {

    double speed = getFuelVelocity(launchSpeed).in(MetersPerSecond);

    double fieldTurretAngle =
        turretAngle.in(Radians)
            + DriveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

    return MetersPerSecond.of(
        speed * Math.cos(hoodToElevation(hoodAngle).in(Radians)) * Math.cos(fieldTurretAngle));
  }

  public LinearVelocity getFuelVelocityY(
      Angle hoodAngle, Angle turretAngle, AngularVelocity launchSpeed) {

    double speed = getFuelVelocity(launchSpeed).in(MetersPerSecond);

    double fieldTurretAngle =
        turretAngle.in(Radians)
            + DriveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

    return MetersPerSecond.of(
        speed * Math.cos(hoodToElevation(hoodAngle).in(Radians)) * Math.sin(fieldTurretAngle));
  }

  public LinearVelocity getFuelVelocityZ(
      Angle hoodAngle, Angle turretAngle, AngularVelocity launchSpeed) {

    double speed = getFuelVelocity(launchSpeed).in(MetersPerSecond);

    return MetersPerSecond.of(speed * Math.sin(hoodToElevation(hoodAngle).in(Radians)));
  }

  public LinearVelocity getFullVelocityX(
      Angle hoodAngle, Angle turretAngle, AngularVelocity launchSpeed) {
    return MetersPerSecond.of(
        getFuelVelocityX(hoodAngle, turretAngle, launchSpeed).in(MetersPerSecond)
            + getTurretVelocityX().in(MetersPerSecond));
  }

  public LinearVelocity getFullVelocityY(
      Angle hoodAngle, Angle turretAngle, AngularVelocity launchSpeed) {
    return MetersPerSecond.of(
        getFuelVelocityY(hoodAngle, turretAngle, launchSpeed).in(MetersPerSecond)
            + getTurretVelocityY().in(MetersPerSecond));
  }

  public LinearVelocity getFullVelocityZ(
      Angle hoodAngle, Angle turretAngle, AngularVelocity launchSpeed) {
    return MetersPerSecond.of(
        getFuelVelocityZ(hoodAngle, turretAngle, launchSpeed).in(MetersPerSecond));
  }

  public Translation2d getShooterFieldPosition() {
    Translation2d robotPosition = DriveSubsystem.getInstance().getRobotPose().getTranslation();

    Translation2d shooterOffset =
        new Translation2d(
            Constants.ShooterConstants.SHOOTER_OFFSET_X.in(Meters),
            Constants.ShooterConstants.SHOOTER_OFFSET_Y.in(Meters));

    double robotHeading = DriveSubsystem.getInstance().getRobotPose().getRotation().getRadians();

    double shooterX =
        shooterOffset.getX() * Math.cos(robotHeading)
            - shooterOffset.getY() * Math.sin(robotHeading);

    double shooterY =
        shooterOffset.getX() * Math.sin(robotHeading)
            + shooterOffset.getY() * Math.cos(robotHeading);

    return new Translation2d(robotPosition.getX() + shooterX, robotPosition.getY() + shooterY);
  }

  public boolean flightNumericalSolver(
      Angle hoodAngle,
      Angle turretAngle,
      AngularVelocity launchSpeed,
      Translation2d target,
      boolean isHub) {
    LinearVelocity fullXVelocity = getFullVelocityX(hoodAngle, turretAngle, launchSpeed);
    LinearVelocity fullYVelocity = getFullVelocityY(hoodAngle, turretAngle, launchSpeed);
    LinearVelocity fullZVelocity = getFullVelocityZ(hoodAngle, turretAngle, launchSpeed);

    Translation2d shooterPosition = getShooterFieldPosition();

    Distance fuelXPos = Meters.of(shooterPosition.getX());
    Distance fuelYPos = Meters.of(shooterPosition.getY());
    Distance fuelZPos = Constants.ShooterConstants.SHOOTER_OFFSET_Z;

    double flightTime = 0;

    while (flightTime < Constants.ShooterConstants.MAX_FLIGHT_TIME) {
      LinearVelocity fuelSpeed =
          MetersPerSecond.of(
              Math.sqrt(
                  Math.pow(fullXVelocity.in(MetersPerSecond), 2)
                      + Math.pow(fullYVelocity.in(MetersPerSecond), 2)
                      + Math.pow(fullZVelocity.in(MetersPerSecond), 2)));

      LinearAcceleration accelerationX =
          MetersPerSecondPerSecond.of(
              -Constants.FieldConstants.DRAG_CONSTANT
                  * fuelSpeed.in(MetersPerSecond)
                  * fullXVelocity.in(MetersPerSecond));

      LinearAcceleration accelerationY =
          MetersPerSecondPerSecond.of(
              -Constants.FieldConstants.DRAG_CONSTANT
                  * fuelSpeed.in(MetersPerSecond)
                  * fullYVelocity.in(MetersPerSecond));

      LinearAcceleration accelerationZ =
          MetersPerSecondPerSecond.of(
              -Constants.FieldConstants.GRAVITY_VALUE
                  - Constants.FieldConstants.DRAG_CONSTANT
                      * fuelSpeed.in(MetersPerSecond)
                      * fullZVelocity.in(MetersPerSecond));

      fullXVelocity =
          MetersPerSecond.of(
              fullXVelocity.in(MetersPerSecond)
                  + accelerationX.in(MetersPerSecondPerSecond)
                      * Constants.FieldConstants.TIME_STEP);
      fullYVelocity =
          MetersPerSecond.of(
              fullYVelocity.in(MetersPerSecond)
                  + accelerationY.in(MetersPerSecondPerSecond)
                      * Constants.FieldConstants.TIME_STEP);
      fullZVelocity =
          MetersPerSecond.of(
              fullZVelocity.in(MetersPerSecond)
                  + accelerationZ.in(MetersPerSecondPerSecond)
                      * Constants.FieldConstants.TIME_STEP);

      fuelXPos =
          Meters.of(
              fuelXPos.in(Meters)
                  + fullXVelocity.in(MetersPerSecond) * Constants.FieldConstants.TIME_STEP);

      fuelYPos =
          Meters.of(
              fuelYPos.in(Meters)
                  + fullYVelocity.in(MetersPerSecond) * Constants.FieldConstants.TIME_STEP);

      fuelZPos =
          Meters.of(
              fuelZPos.in(Meters)
                  + fullZVelocity.in(MetersPerSecond) * Constants.FieldConstants.TIME_STEP);

      flightTime += Constants.FieldConstants.TIME_STEP;

      if (fuelZPos.in(Meters) > Constants.FieldConstants.MAX_BALL_Y_POS.getAsDouble()) {
        return false;
      }

      if (fuelZPos.in(Meters) < Constants.ShooterConstants.SHOOTER_OFFSET_Z.in(Meters) - .01
          && isHub) {
        return false;
      }

      Distance height = (isHub) ? Meters.of(Constants.FieldConstants.HUB_Y_POS) : Meters.of(0);

      Distance distanceToTarget =
          Meters.of(
              Math.sqrt(
                  Math.pow((target.getX() - fuelXPos.in(Meters)), 2)
                      + Math.pow((target.getY() - fuelYPos.in(Meters)), 2)
                      + Math.pow((height.in(Meters) - fuelZPos.in(Meters)), 2)));

      if (distanceToTarget.in(Meters) <= Constants.FieldConstants.HUB_WIDTH.in(Meters) / 2.0
          && fullZVelocity.in(MetersPerSecond) < 0) {
        return true;
      }
    }
    return false;
  }

  public ShotSolution findOptimalSolution(Translation2d target, boolean isHub) {
    ShotSolution bestShot = null;

    if (DriveSubsystem.getInstance().isUnderTrench()) {
      return new ShotSolution(
          Constants.ShooterConstants.TURRET_MINIMUM_ANGLE,
          Constants.ShooterConstants.HOOD_MINIMUM_ANGLE,
          Constants.ShooterConstants.MIN_SHOOTER_VELOCITY);
    }

    for (Angle hoodAngle = Constants.ShooterConstants.HOOD_MINIMUM_ANGLE;
        hoodAngle.in(Degrees) <= Constants.ShooterConstants.HOOD_MAX_ANGLE.in(Degrees);
        hoodAngle = hoodAngle.plus(Constants.ShooterConstants.HOOD_ANGLE_STEP)) {

      for (Angle turretAngle = Constants.ShooterConstants.TURRET_MINIMUM_ANGLE;
          turretAngle.in(Degrees) <= Constants.ShooterConstants.TURRET_MAX_ANGLE.in(Degrees);
          turretAngle = turretAngle.plus(Constants.ShooterConstants.TURRET_ANGLE_STEP)) {

        for (AngularVelocity shooterVelocity = Constants.ShooterConstants.MIN_SHOOTER_VELOCITY;
            shooterVelocity.in(RadiansPerSecond)
                <= Constants.ShooterConstants.MAX_SHOOTER_VELOCITY.in(RadiansPerSecond);
            shooterVelocity =
                shooterVelocity.plus(Constants.ShooterConstants.SHOOTER_VELOCITY_STEP)) {

          if (!flightNumericalSolver(hoodAngle, turretAngle, shooterVelocity, target, isHub)) {
            continue;
          }

          if (bestShot == null
              || shooterVelocity.in(RadiansPerSecond)
                  < bestShot.shooterVelocity().in(RadiansPerSecond)) {

            bestShot = new ShotSolution(turretAngle, hoodAngle, shooterVelocity);
          }
        }
      }
    }

    return bestShot;
  }

  public LinearVelocity getFuelVelocity(AngularVelocity shooterVelocity) {

    return MetersPerSecond.of(
        shooterVelocity.in(RotationsPerSecond)
            * Constants.ShooterConstants.BALL_METERS_PER_MOTOR_ROTATION);
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

  public boolean atGoodShooterVelocity(AngularVelocity velocity) {
    return (Math.abs(velocity.in(RotationsPerSecond))
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

  public void shoot(Translation2d target, boolean isHub) {
    setTurretAngle(getInstance().shot.turretAngle());
    setHoodAngle(getInstance().shot.hoodAngle());
    setShooterVelocity(getInstance().shot.shooterVelocity());
  }

  public void unwindTurret() {
    getInstance()
        .m_turretMotor
        .setControl(getInstance().m_positionRequest.withPosition(Degrees.of(0)));
  }

  public boolean finishedUnwind() {
    double current = getInstance().m_turretMotor.getPosition().getValue().in(Degrees);

    double target = Constants.ShooterConstants.TURRET_UNWIND_ANGLE.in(Degrees);

    return Math.abs(current - target) <= 2;
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
    return (getInstance().atGoodHoodAngle(getInstance().shot.hoodAngle())
        && getInstance().atGoodShooterVelocity(getInstance().shot.shooterVelocity())
        && DriveSubsystem.getInstance().atGoodShootingPosition()
        && getInstance().atGoodTurretAngle(getInstance().shot.turretAngle()));
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
    target = getInstance().getShootingTarget();
    hub =
        (target == Constants.FieldConstants.BLUE_HUB_COORDINATES
                || target == Constants.FieldConstants.RED_HUB_COORDINATES)
            ? true
            : false;
    shot = findOptimalSolution(target, hub);
    Logger.recordOutput("ShooterSubsystem/State", getState().toString());
    Logger.recordOutput("ShooterSubsystem/HoodAngle", m_hoodMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("ShooterSubsystem/DesiredHoodAngle", shot.hoodAngle());
    Logger.recordOutput("ShooterSubsystem/DesiredShooterVelocity", shot.shooterVelocity());
    Logger.recordOutput("ShooterSubsystem/DesiredTurretAngle", shot.turretAngle());
    Logger.recordOutput("ShooterSubsystem/ShooterTarget", getShootingTarget());
    Logger.recordOutput(
        "ShooterSubsystem/ShooterVelocity", m_flywheelLeaderMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "ShooterSubsystem/TurretAngle", m_turretMotor.getPosition().getValueAsDouble());
  }
}
