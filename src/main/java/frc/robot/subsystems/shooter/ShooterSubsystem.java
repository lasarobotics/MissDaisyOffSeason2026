// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
        if (getInstance().shot != null) {
          getInstance().setTurretAngle(getInstance().shot.turretAngle());
          getInstance().setHoodAngle(getInstance().shot.hoodAngle());
          getInstance().setShooterVelocity(Constants.ShooterConstants.FLYWHEEL_REST_SPEED);
        }
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
          Logger.recordOutput("Debug/GoodShootPos", true);
          if ((getInstance().atUnwindAngle() || getInstance().m_isUnwinding)
              && !getInstance().finishedUnwind()) {
            Logger.recordOutput("Debug/Shooting", false);
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
            Logger.recordOutput("Debug/Shooting", true);
          }
        } else {
          Logger.recordOutput("Debug/GoodShootPos", false);
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
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ShooterConstants.MAX_TURRET_ROTS;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        -Constants.ShooterConstants.MAX_TURRET_ROTS;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    m_turretMotor.getConfigurator().apply(turretConfig);

    updateTurretPosition();
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

  public static Angle elevationToHood(Angle elevation) {
    return elevation
        .minus(Constants.ShooterConstants.HOOD_ELEVATION_OFFSET)
        .div(Constants.ShooterConstants.HOOD_ELEVATION_SCALAR);
  }

  public AngularVelocity toShooterVelocity(LinearVelocity ballSpeed) {
    return RotationsPerSecond.of(
        ballSpeed.in(MetersPerSecond) / Constants.ShooterConstants.BALL_METERS_PER_MOTOR_ROTATION);
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

  public record FlightResult(Distance height, LinearVelocity verticalVelocity) {}

  private FlightResult simulatePlanarFlight(
      LinearVelocity horizontalSpeed,
      LinearVelocity verticalSpeed,
      Distance startHeight,
      Distance targetDistance) {

    double timeStep = Constants.FieldConstants.TIME_STEP;
    double drag = Constants.FieldConstants.DRAG_CONSTANT;
    double gravity = Constants.FieldConstants.GRAVITY_VALUE;
    double ceiling = Constants.FieldConstants.MAX_BALL_Y_POS.getAsDouble();
    double maxFlightTime = Constants.ShooterConstants.MAX_FLIGHT_TIME;
    double targetDistanceMeters = targetDistance.in(Meters);

    double distance = 0;
    double height = startHeight.in(Meters);
    double horizontalVelocity = horizontalSpeed.in(MetersPerSecond);
    double verticalVelocity = verticalSpeed.in(MetersPerSecond);
    double flightTime = 0;

    while (flightTime < maxFlightTime) {
      double speed = Math.hypot(horizontalVelocity, verticalVelocity);
      double previousDistance = distance;
      double previousHeight = height;

      horizontalVelocity += -drag * speed * horizontalVelocity * timeStep;
      verticalVelocity += (-gravity - drag * speed * verticalVelocity) * timeStep;
      distance += horizontalVelocity * timeStep;
      height += verticalVelocity * timeStep;
      flightTime += timeStep;

      if (height > ceiling) {
        return new FlightResult(Meters.of(Double.POSITIVE_INFINITY), MetersPerSecond.of(0));
      }

      if (horizontalVelocity <= 0) {
        return new FlightResult(Meters.of(Double.NEGATIVE_INFINITY), MetersPerSecond.of(0));
      }

      if (distance >= targetDistanceMeters) {
        double fraction = (targetDistanceMeters - previousDistance) / (distance - previousDistance);
        return new FlightResult(
            Meters.of(previousHeight + fraction * (height - previousHeight)),
            MetersPerSecond.of(verticalVelocity));
      }
    }

    return new FlightResult(Meters.of(Double.NEGATIVE_INFINITY), MetersPerSecond.of(0));
  }

  private LinearVelocity solveLaunchSpeed(
      Angle elevation,
      Distance startHeight,
      Distance targetDistance,
      Distance targetHeight,
      LinearVelocity maxSpeed) {

    double cosine = Math.cos(elevation.in(Radians));
    double sine = Math.sin(elevation.in(Radians));
    double targetHeightMeters = targetHeight.in(Meters);

    double low = 0;
    double high = maxSpeed.in(MetersPerSecond);

    for (int i = 0; i < Constants.ShooterConstants.SPEED_BISECTION_STEPS; i++) {
      double mid = 0.5 * (low + high);
      FlightResult flight =
          simulatePlanarFlight(
              MetersPerSecond.of(mid * cosine),
              MetersPerSecond.of(mid * sine),
              startHeight,
              targetDistance);

      if (flight.height().in(Meters) < targetHeightMeters) {
        low = mid;
      } else {
        high = mid;
      }
    }

    FlightResult flight =
        simulatePlanarFlight(
            MetersPerSecond.of(high * cosine),
            MetersPerSecond.of(high * sine),
            startHeight,
            targetDistance);

    if (Math.abs(flight.height().in(Meters) - targetHeightMeters)
            > Constants.FieldConstants.HUB_WIDTH.in(Meters) / 2.0
        || flight.verticalVelocity().in(MetersPerSecond) >= 0) {
      return null;
    }

    return MetersPerSecond.of(high);
  }

  private Angle resolveTurretAngle(Angle fieldAzimuth, Angle robotHeading) {
    double minimum = Constants.ShooterConstants.TURRET_MINIMUM_ANGLE.in(Degrees);
    double maximum = Constants.ShooterConstants.TURRET_MAX_ANGLE.in(Degrees);

    double candidate = fieldAzimuth.minus(robotHeading).in(Degrees);
    while (candidate < minimum) {
      candidate += 360;
    }
    while (candidate >= minimum + 360) {
      candidate -= 360;
    }

    if (candidate <= maximum) {
      return Degrees.of(candidate);
    }

    candidate -= 360;
    if (candidate >= minimum) {
      return Degrees.of(candidate);
    }

    return null;
  }

  public ShotSolution findOptimalSolution(Translation2d target, boolean isHub) {
    if (DriveSubsystem.getInstance().isUnderTrench()) {
      return new ShotSolution(
          Constants.ShooterConstants.TURRET_MINIMUM_ANGLE,
          Constants.ShooterConstants.HOOD_MINIMUM_ANGLE,
          Constants.ShooterConstants.MIN_SHOOTER_VELOCITY);
    }

    Angle robotHeading = DriveSubsystem.getInstance().getRobotPose().getRotation().getMeasure();
    Translation2d launchPosition = getShooterFieldPosition();
    LinearVelocity turretVelocityX = getTurretVelocityX();
    LinearVelocity turretVelocityY = getTurretVelocityY();

    Translation2d toTarget = target.minus(launchPosition);
    Distance targetDistance = Meters.of(toTarget.getNorm());

    if (targetDistance.in(Meters) <= 0) {
      return null;
    }

    Rotation2d aim = toTarget.getAngle();

    Distance startHeight = Constants.ShooterConstants.SHOOTER_OFFSET_Z;
    Distance targetHeight = isHub ? Meters.of(Constants.FieldConstants.HUB_Y_POS) : Meters.of(0);

    LinearVelocity maxSpeed =
        getFuelVelocity(Constants.ShooterConstants.MAX_SHOOTER_VELOCITY)
            .plus(
                MetersPerSecond.of(
                    Math.hypot(
                        turretVelocityX.in(MetersPerSecond), turretVelocityY.in(MetersPerSecond))));

    ShotSolution bestShot = null;

    for (Angle hoodAngle = Constants.ShooterConstants.HOOD_MINIMUM_ANGLE;
        hoodAngle.in(Degrees) <= Constants.ShooterConstants.HOOD_MAX_ANGLE.in(Degrees);
        hoodAngle = hoodAngle.plus(Constants.ShooterConstants.HOOD_ANGLE_STEP)) {

      Angle elevation = hoodToElevation(hoodAngle);
      if (elevation.in(Radians) <= 0 || elevation.in(Radians) >= Math.PI / 2.0) {
        continue;
      }

      LinearVelocity fullSpeed =
          solveLaunchSpeed(elevation, startHeight, targetDistance, targetHeight, maxSpeed);
      if (fullSpeed == null) {
        continue;
      }

      LinearVelocity fullHorizontal = fullSpeed.times(Math.cos(elevation.in(Radians)));

      LinearVelocity muzzleX = fullHorizontal.times(aim.getCos()).minus(turretVelocityX);
      LinearVelocity muzzleY = fullHorizontal.times(aim.getSin()).minus(turretVelocityY);
      LinearVelocity muzzleZ = fullSpeed.times(Math.sin(elevation.in(Radians)));

      LinearVelocity muzzleHorizontal =
          MetersPerSecond.of(Math.hypot(muzzleX.in(MetersPerSecond), muzzleY.in(MetersPerSecond)));

      Angle solvedHood =
          elevationToHood(
              Radians.of(
                  Math.atan2(muzzleZ.in(MetersPerSecond), muzzleHorizontal.in(MetersPerSecond))));

      if (solvedHood.in(Degrees) < Constants.ShooterConstants.HOOD_MINIMUM_ANGLE.in(Degrees)
          || solvedHood.in(Degrees) > Constants.ShooterConstants.HOOD_MAX_ANGLE.in(Degrees)) {
        continue;
      }

      Angle solvedTurret =
          resolveTurretAngle(
              Radians.of(Math.atan2(muzzleY.in(MetersPerSecond), muzzleX.in(MetersPerSecond))),
              robotHeading);
      if (solvedTurret == null) {
        continue;
      }

      AngularVelocity solvedVelocity =
          toShooterVelocity(
              MetersPerSecond.of(
                  Math.hypot(muzzleHorizontal.in(MetersPerSecond), muzzleZ.in(MetersPerSecond))));

      if (solvedVelocity.in(RotationsPerSecond)
              < Constants.ShooterConstants.MIN_SHOOTER_VELOCITY.in(RotationsPerSecond)
          || solvedVelocity.in(RotationsPerSecond)
              > Constants.ShooterConstants.MAX_SHOOTER_VELOCITY.in(RotationsPerSecond)) {
        continue;
      }

      if (bestShot == null
          || solvedVelocity.in(RotationsPerSecond)
              < bestShot.shooterVelocity().in(RotationsPerSecond)) {
        bestShot = new ShotSolution(solvedTurret, solvedHood, solvedVelocity);
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
    if (getInstance().shot == null) return;
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
          return Constants.FieldConstants.BLUE_AZ_PASS_RIGHT;
        } else {
          return Constants.FieldConstants.BLUE_AZ_PASS_LEFT;
        }
      }
      if (robotTranslation.getX() > Constants.FieldConstants.RED_ZONE_X) {
        if (robotTranslation.getY() < Constants.FieldConstants.HALF_FIELD_Y_POS) {
          return Constants.FieldConstants.BLUE_NZ_PASS_RIGHT;
        } else {
          return Constants.FieldConstants.BLUE_NZ_PASS_LEFT;
        }
      }
    } else {
      if (robotTranslation.getX() > Constants.FieldConstants.RED_ZONE_X) {
        return Constants.FieldConstants.RED_HUB_COORDINATES;
      }
      if (robotTranslation.getX() > Constants.FieldConstants.BLUE_ZONE_X
          && robotTranslation.getX() < Constants.FieldConstants.RED_ZONE_X) {
        if (robotTranslation.getY() < Constants.FieldConstants.HALF_FIELD_Y_POS) {
          return Constants.FieldConstants.RED_AZ_PASS_RIGHT;
        } else {
          return Constants.FieldConstants.RED_AZ_PASS_LEFT;
        }
      }
      if (robotTranslation.getX() < Constants.FieldConstants.BLUE_ZONE_X) {
        if (robotTranslation.getY() < Constants.FieldConstants.HALF_FIELD_Y_POS) {
          return Constants.FieldConstants.RED_NZ_PASS_RIGHT;
        } else {
          return Constants.FieldConstants.RED_NZ_PASS_LEFT;
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
    if (getInstance().shot == null) return false;
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
    for (int i = 0; i < Constants.ShooterConstants.ENCODER_TEETH_ONE; i++) {
      encoderOnePossible[i] =
          (i + (encoderAPosition / 360))
              * ((double) Constants.ShooterConstants.ENCODER_TEETH_ONE
                  / Constants.ShooterConstants.TURRET_GEAR_TEETH);
    }
    // for encoder two
    for (int i = 0; i < Constants.ShooterConstants.ENCODER_TEETH_TWO; i++) {
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
    Logger.recordOutput("ShooterSubsystem/isHub", hub);
    Logger.recordOutput("ShooterSubsystem/State", getState().toString());
    Logger.recordOutput("ShooterSubsystem/HoodAngle", m_hoodMotor.getPosition().getValueAsDouble());
    if (getInstance().shot != null) {
      Logger.recordOutput("ShooterSubsystem/DesiredHoodAngle", shot.hoodAngle());
      Logger.recordOutput("ShooterSubsystem/DesiredShooterVelocity", shot.shooterVelocity());
      Logger.recordOutput("ShooterSubsystem/DesiredTurretAngle", shot.turretAngle());
      Logger.recordOutput(
          "ShooterSubsystem/TurretPos",
          new Pose2d(
              DriveSubsystem.getInstance().getRobotPose().getTranslation(),
              new Rotation2d(shot.turretAngle())));
    }
    Logger.recordOutput("ShooterSubsystem/ShooterTarget", getShootingTarget());
    Logger.recordOutput(
        "ShooterSubsystem/ShooterVelocity", m_flywheelLeaderMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "ShooterSubsystem/TurretAngle", m_turretMotor.getPosition().getValueAsDouble());
  }
}
