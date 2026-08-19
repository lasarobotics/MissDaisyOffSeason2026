// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

  public enum DriveStates implements SystemState {
    AUTO {

      @Override
      public SystemState nextState() {
        return this;
      }
    },

    DRIVER_CONTROL {
      @Override
      public void execute() {
        AngularVelocity rotationRate =
            Constants.DriveConstants.MAX_ANGULAR_RATE
                .times(-s_rotateRequest.getAsDouble())
                .times(s_currentSpeedScalar);
        s_drivetrain.setControl(
            s_drive
                .withVelocityX(
                    Constants.DriveConstants.MAX_SPEED
                        .times(
                            -s_strafeRequest.getAsDouble()
                                * Math.abs(s_strafeRequest.getAsDouble()))
                        .times(s_currentSpeedScalar))
                .withVelocityY(
                    Constants.DriveConstants.MAX_SPEED
                        .times(
                            -s_driveRequest.getAsDouble() * Math.abs(s_driveRequest.getAsDouble()))
                        .times(s_currentSpeedScalar))
                .withRotationalRate(rotationRate));
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    },

    CLIMB_ALIGN {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        double currentRotation = s_drivetrain.getState().Pose.getRotation().getRadians();
        double pidOutputAngle =
            getInstance()
                .m_rotationPIDController
                .calculate(
                    currentRotation,
                    DriverStation.getAlliance().get() == Alliance.Blue ? 0 : Math.PI);
        double pidInput =
            Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle).in(RadiansPerSecond);
        pidInput = pidInput > 0 ? Math.min(pidInput, 8.0) : Math.max(pidInput, -8.0);
        s_drivetrain.setControl(
            s_drive
                .withVelocityX(
                    Constants.DriveConstants.MAX_SPEED
                        .times(-s_strafeRequest.getAsDouble())
                        .times(s_currentSpeedScalar))
                .withVelocityY(
                    Constants.DriveConstants.MAX_SPEED
                        .times(-s_driveRequest.getAsDouble())
                        .times(s_currentSpeedScalar))
                .withRotationalRate(pidInput));
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    },

    UNWIND {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return this;
      }
    }
  }

  private static DriveSubsystem s_driveInstance;
  private DriveStates m_requestedState;
  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;
  private static DoubleSupplier s_driveRequest;
  private static DoubleSupplier s_strafeRequest;
  private static DoubleSupplier s_rotateRequest;
  private static SwerveRequest.FieldCentric s_autoDrive;
  private PIDController m_rotationPIDController;
  private PIDController m_translationPIDController;
  private static double s_currentSpeedScalar;

  private boolean m_shouldAlign;

  public DriveSubsystem() {
    super(DriveStates.DRIVER_CONTROL);
    s_currentSpeedScalar = Constants.DriveConstants.FAST_SPEED_SCALAR;

    m_requestedState = DriveStates.DRIVER_CONTROL;
  }

  public void setState(DriveStates state) {
    getInstance().m_requestedState = state;
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  public void setShouldAlign(Boolean value) {
    getInstance().m_shouldAlign = value;
  }

  public Translation2d getClosestClimbPos() {
    Translation2d robotTranslation = s_drivetrain.getState().Pose.getTranslation();

    return (robotTranslation.getDistance(Constants.ClimbConstants.CLIMB_POS_LEFT)
            > robotTranslation.getDistance(Constants.ClimbConstants.CLIMB_POS_RIGHT))
        ? Constants.ClimbConstants.CLIMB_POS_RIGHT
        : Constants.ClimbConstants.CLIMB_POS_LEFT;
  }

  public Pose2d getRobotPose() {
    return s_drivetrain.getState().Pose;
  }

  public Translation2d getShooterTranslation() {
    Pose2d robotPose = getRobotPose();
    double shooterX = robotPose.getX() - Constants.ShooterConstants.SHOOTER_OFFSET_X.in(Meters);
    double shooterY = robotPose.getY() - Constants.ShooterConstants.SHOOTER_OFFSET_Y.in(Meters);
    return new Translation2d(shooterX, shooterY);
  }

  public Distance getDistanceToHubX() {
    Translation2d shooterTranslation = getShooterTranslation();
    Translation2d translationDiff;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      translationDiff = shooterTranslation.minus(Constants.FieldConstants.BLUE_HUB_COORDINATES);
    } else {
      translationDiff = shooterTranslation.minus(Constants.FieldConstants.RED_HUB_COORDINATES);
    }

    return Meters.of(
        Math.sqrt(Math.pow(translationDiff.getX(), 2)) + Math.pow(translationDiff.getY(), 2));
  }

  public Distance getDistance(Translation2d target) {
    Translation2d robotTranslation = getRobotPose().getTranslation();
    return Meters.of(
        Math.sqrt(
            Math.pow(robotTranslation.getMeasureX().minus(target.getMeasureX()).in(Meters), 2)
                + Math.pow(
                    robotTranslation.getMeasureY().minus(target.getMeasureY()).in(Meters), 2)));
  }

  public boolean isUnderTrench() {
    return true;
  }

  public void goTo(
      Pose2d location,
      double maxVelocity,
      double exitVelocity,
      double targetRotation,
      double maxRotationRate) {

    Pose2d currentPose = s_drivetrain.getState().Pose;
    Translation2d positionDiff = location.getTranslation().minus(currentPose.getTranslation());
    double distance = currentPose.getTranslation().getDistance(location.getTranslation());

    double pidOutput = -m_translationPIDController.calculate(distance, 0);

    Rotation2d travelDirection = positionDiff.getAngle();

    double trueVelocity = MathUtil.clamp(Math.max(pidOutput, exitVelocity) + 0.1, 0, maxVelocity);

    double xControl = trueVelocity * travelDirection.getCos();
    double yControl = trueVelocity * travelDirection.getSin();

    double rotationRate =
        MathUtil.clamp(
            m_rotationPIDController.calculate(
                currentPose.getRotation().getRadians(), targetRotation),
            -maxRotationRate,
            maxRotationRate);

    s_drivetrain.setControl(
        s_autoDrive
            .withVelocityX(MetersPerSecond.of(xControl))
            .withVelocityY(MetersPerSecond.of(yControl))
            .withRotationalRate(rotationRate));
  }

  public void configureBindings(
      DoubleSupplier strafeRequest, DoubleSupplier driveRequest, DoubleSupplier rotateRequest) {
    s_strafeRequest = strafeRequest;
    s_driveRequest = driveRequest;
    s_rotateRequest = rotateRequest;
  }

  public static CommandSwerveDrivetrain getDrivetrain() {
    return s_drivetrain;
  }

  public static boolean isCommandedMoving() {
    ChassisSpeeds speeds = getSpeeds();
    return Math.abs(speeds.vxMetersPerSecond) > Constants.DriveConstants.MOVEMENT_THRESHOLD
        || Math.abs(speeds.vyMetersPerSecond) > Constants.DriveConstants.MOVEMENT_THRESHOLD
        || Math.abs(speeds.omegaRadiansPerSecond) > Constants.DriveConstants.MOVEMENT_THRESHOLD;
  }

  public static ChassisSpeeds getSpeeds() {
    return s_drivetrain.getState().Speeds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() {}
}
