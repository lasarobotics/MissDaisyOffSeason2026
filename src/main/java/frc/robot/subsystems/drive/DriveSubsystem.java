// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
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
import frc.robot.HeadHoncho;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends StateMachine {

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

    s_drivetrain = TunerConstants.createDrivetrain();
    s_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(
                Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    setPerspective();
  }

  public void setPerspective() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      }
      if (ally.get() == Alliance.Blue) {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
    }
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
    Translation2d robotTranslation = getInstance().getRobotPose().getTranslation();
    if ((HeadHoncho.getInstance()
                    .numberWithinThreshold(
                        Constants.FieldConstants.BLUE_TRENCH_LEFT_CENTER.getX(),
                        robotTranslation.getX(),
                        Constants.FieldConstants.TRENCH_THRESHOLD)
                || HeadHoncho.getInstance()
                    .numberWithinThreshold(
                        Constants.FieldConstants.RED_TRENCH_RIGHT_CENTER.getX(),
                        robotTranslation.getX(),
                        Constants.FieldConstants.TRENCH_THRESHOLD))
            && (HeadHoncho.getInstance()
                    .numberWithinThreshold(
                        Constants.FieldConstants.BLUE_TRENCH_LEFT_CENTER.getY(),
                        robotTranslation.getY(),
                        Constants.FieldConstants.TRENCH_THRESHOLD)
                || HeadHoncho.getInstance()
                    .numberWithinThreshold(
                        Constants.FieldConstants.BLUE_TRENCH_RIGHT_CENTER.getY(),
                        robotTranslation.getY(),
                        Constants.FieldConstants.TRENCH_THRESHOLD)
                || HeadHoncho.getInstance()
                    .numberWithinThreshold(
                        Constants.FieldConstants.RED_TRENCH_LEFT_LEFT.getX(),
                        robotTranslation.getX(),
                        Constants.FieldConstants.TRENCH_THRESHOLD))
        || HeadHoncho.getInstance()
            .numberWithinThreshold(
                Constants.FieldConstants.RED_TRENCH_RIGHT_CENTER.getY(),
                robotTranslation.getY(),
                Constants.FieldConstants.TRENCH_THRESHOLD)) {
      return true;
    }

    return false;
  }

  public boolean atGoodShootingPosition() {
    Translation2d robotTranslation = getInstance().getRobotPose().getTranslation();
    if (isUnderTrench()
        || ((robotTranslation.getX() <= Constants.FieldConstants.BLUE_TOWER_LEFT.getX()
                || robotTranslation.getX() >= Constants.FieldConstants.RED_TOWER_LEFT.getX())
            && ((robotTranslation.getY() >= Constants.FieldConstants.RED_TOWER_LEFT.getY()
                    && robotTranslation.getY() <= Constants.FieldConstants.RED_TOWER_RIGHT.getY())
                || (robotTranslation.getY() >= Constants.FieldConstants.BLUE_TOWER_LEFT.getY()
                    && robotTranslation.getY()
                        <= Constants.FieldConstants.BLUE_TOWER_RIGHT.getY())))) {
      return true;
    }
    return false;
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
    Logger.recordOutput("DriveSubsystem/Pose", getInstance().getRobotPose());
    Logger.recordOutput("DriveSubsystem/State", getState().toString());
  }
}
