// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends StateMachine {

  public enum DriveStates implements SystemState {
    AUTO {
      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) {
          return DRIVER_CONTROL;
        }
        return AUTO;
      }
    },
    DRIVER_CONTROL {

      @Override
      public void execute() {
        AngularVelocity rotationRate =
            Constants.DriveConstants.MAX_ANGULAR_RATE.times(
                -getInstance().m_rotateRequest.getAsDouble());

        s_drivetrain.setControl(
            s_drive
                .withVelocityX(
                    Constants.DriveConstants.MAX_SPEED
                        .times(
                            -getInstance().m_strafeRequest.getAsDouble()
                                * Math.abs(getInstance().m_strafeRequest.getAsDouble()))
                        .times(getInstance().m_currentSpeedScalar))
                .withVelocityY(
                    Constants.DriveConstants.MAX_SPEED
                        .times(
                            -getInstance().m_driveRequest.getAsDouble()
                                * Math.abs(getInstance().m_driveRequest.getAsDouble()))
                        .times(getInstance().m_currentSpeedScalar))
                .withRotationalRate(rotationRate.times(getInstance().m_currentSpeedScalar)));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static DriveSubsystem s_driveInstance;
  private DoubleSupplier m_driveRequest;
  private DoubleSupplier m_strafeRequest;
  private DoubleSupplier m_rotateRequest;
  private DriveStates m_selectedState;
  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;
  private BooleanSupplier m_slowdownRequest;
  private double m_currentSpeedScalar;
  private Translation2d m_limeEstimate;

  // private PIDController m_rotationPIDController;

  public DriveSubsystem() {
    super(DriveStates.DRIVER_CONTROL);
    setState(DriveStates.DRIVER_CONTROL);
    s_drivetrain = TunerConstants.createDrivetrain();
    s_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(
                Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    // m_rotationPIDController =
    //     new PIDController(
    //         Constants.DriveConstants.TURN_P,
    //         Constants.DriveConstants.TURN_I,
    //         Constants.DriveConstants.TURN_D);
    // m_rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
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

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  public void configureBindings(
      DoubleSupplier strafeRequest,
      DoubleSupplier driveRequest,
      DoubleSupplier rotateRequest,
      BooleanSupplier slowdownRequest) {
    m_strafeRequest = strafeRequest;
    m_driveRequest = driveRequest;
    m_rotateRequest = rotateRequest;
    m_slowdownRequest = slowdownRequest;
  }

  public void setState(DriveStates state) {
    m_selectedState = state;
  }

  public Pose2d getPose() {
    return s_drivetrain.getState().Pose;
  }

  public ChassisSpeeds getSpeeds() {
    return s_drivetrain.getState().Speeds;
  }

  public Translation2d getTranslation2d() {
    return s_drivetrain.getState().Pose.getTranslation();
  }

  public Translation2d getLimelightTranslation2d() {
    return m_limeEstimate;
  }

  private LimelightHelpers.PoseEstimate getFilteredPoseEstimate() {
    LimelightHelpers.PoseEstimate pose_estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    if (pose_estimate == null) {
      return null;
    }

    if (Math.abs(s_drivetrain.getState().Speeds.omegaRadiansPerSecond) > 2 * Math.PI) {
      return null;
    }

    if (pose_estimate.tagCount == 0) {
      return null;
    }

    if (Double.isNaN(pose_estimate.pose.getX())
        || Double.isNaN(pose_estimate.pose.getY())
        || Double.isNaN(pose_estimate.pose.toPose2d().getRotation().getDegrees())) {
      return null;
    }

    // filtering for unreasonable poses
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // welded perimeter field is slightly larger
    // 16.540988 meters x
    // 8.069326 meters y
    // bump is 16 cm off the ground
    // and anything above 25cm is probably insane airtime & unreliable
    if (pose_estimate.pose.getX() < 0
        || pose_estimate.pose.getX() > 16.540988
        || pose_estimate.pose.getY() < 0
        || pose_estimate.pose.getY() > 8.069326
        || pose_estimate.pose.getZ() < -0.05
        || pose_estimate.pose.getZ() > 0.25) {
      return null;
    }

    // aggressive filtering for one tag
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
    if (pose_estimate.tagCount == 1 && pose_estimate.rawFiducials.length == 1) {
      RawFiducial tag = pose_estimate.rawFiducials[0];
      // ignore anything that has too high ambiguity
      if (tag.ambiguity > Constants.DriveConstants.SINGLE_TAG_AMBIGUITY_CUTOFF) {
        return null;
      }
      // we outright reject anything further than a certain distance
      if (tag.distToCamera > Constants.DriveConstants.SINGLE_TAG_DISTANCE_CUTOFF) {
        return null;
      }
    }
    return pose_estimate;
  }

  @Override
  public void periodic() {

    m_currentSpeedScalar =
        m_slowdownRequest.getAsBoolean() ? Constants.DriveConstants.SLOWDOWN_SPEED : 1;
    Logger.recordOutput("DriveSubsystem/Pose", s_drivetrain.getState().Pose);

    LimelightHelpers.PoseEstimate limelightEstimate = getFilteredPoseEstimate();
    if (limelightEstimate != null && limelightEstimate.tagCount > 0) {

      // since the limelight is on the turret, we have to translate and rotate the pose
      m_limeEstimate = limelightEstimate.pose.toPose2d().getTranslation();
      Translation2d turretOffset =
          new Translation2d(
              Constants.ShooterConstants.SHOOTER_OFFSET_X,
              Constants.ShooterConstants.SHOOTER_OFFSET_Y);
      Rotation2d turretAngleOffset =
          new Rotation2d(ShooterSubsystem.getInstance().getTurretRotation() * 2 * Math.PI);
      Transform2d turretToCenter = new Transform2d(turretOffset, turretAngleOffset);
      Pose2d estimatedTurretPose = limelightEstimate.pose.toPose2d();
      Pose2d truePose = estimatedTurretPose.transformBy(turretToCenter.inverse());

      if (!DriverStation.isDisabled()) {
        s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
        s_drivetrain.addVisionMeasurement(
            truePose, Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));
        Logger.recordOutput(getName() + "/LimeLight Pose", truePose);
      } else {
        s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(3, 3, 3));
        s_drivetrain.addVisionMeasurement(
            truePose, Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));
      }
      Logger.recordOutput(getName() + "/LimeLight Pose", truePose);
    }
    if (limelightEstimate != null) {
      Logger.recordOutput(getName() + "/TagCount", limelightEstimate.tagCount);
    }
  }
}
