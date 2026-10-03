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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.Constants.BLineConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

import frc.robot.generated.TunerConstants;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

// BLine imports
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterStates;

public class DriveSubsystem extends StateMachine {
  public enum DriveStates implements SystemState {

    REST {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_driveState;
      }
    },

    DRIVER_CONTROL {
      @Override
      public void initialize() {}

      @Override
      public void execute() {

        AngularVelocity rotationRate =
            Constants.DriveConstants.MAX_ANGULAR_RATE.times(
                -getInstance().m_rotateRequest.getAsDouble());

        getInstance()
            .m_driveTrain
            .setControl(
                getInstance()
                    .m_drive
                    .withVelocityX(
                        Constants.DriveConstants.MAX_SPEED.times(
                            -getInstance().m_strafeRequest.getAsDouble()
                                * Math.abs(
                                    getInstance().m_strafeRequest.getAsDouble())))
                    .withVelocityY(
                        Constants.DriveConstants.MAX_SPEED.times(
                            -getInstance().m_driveRequest.getAsDouble()
                                * Math.abs(
                                    getInstance().m_driveRequest.getAsDouble())))
                    .withRotationalRate(rotationRate));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_driveState;
      }
    }
  }

  private static DriveSubsystem s_driveInstance;
  private DriveStates m_driveState = DriveStates.REST;

  private java.util.function.DoubleSupplier m_driveRequest;
  private java.util.function.DoubleSupplier m_strafeRequest;
  private java.util.function.DoubleSupplier m_rotateRequest;

  private CommandSwerveDrivetrain m_driveTrain;
  private SwerveRequest.FieldCentric m_drive;
  private Translation2d m_hubPos;

  private final SwerveRequest.ApplyRobotSpeeds m_blineDriveRequest =
      new SwerveRequest.ApplyRobotSpeeds();
  private final FollowPath.Builder m_blinePathBuilder;
  private final SendableChooser<Command> m_autoChooser =
      new SendableChooser<>();

  public DriveSubsystem() {

    super(DriveStates.REST);

    m_driveTrain = TunerConstants.createDrivetrain();
    setPerspective();

    m_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(
                Constants.DriveConstants.MAX_SPEED.times(
                    Constants.DriveConstants.DEADBAND_SCALAR))
            .withRotationalDeadband(
                Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1))
            .withDriveRequestType(
                DriveRequestType.Velocity)
            .withSteerRequestType(
                SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(
                ForwardPerspectiveValue.OperatorPerspective);

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            BLineConstants.MAX_VELOCITY_MPS,
            BLineConstants.MAX_ACCELERATION_MPS2,
            BLineConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC,
            BLineConstants.MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2,
            BLineConstants.END_TRANSLATION_TOLERANCE_METERS,
            BLineConstants.END_ROTATION_TOLERANCE_DEG,
            BLineConstants.INTERMEDIATE_HANDOFF_RADIUS_METERS
        ));

    m_blinePathBuilder =
        new FollowPath.Builder(
            this,
            this::getPose,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new PIDController(
                BLineConstants.TRANSLATION_KP,
                BLineConstants.TRANSLATION_KI,
                BLineConstants.TRANSLATION_KD),
            new PIDController(
                BLineConstants.ROTATION_KP,
                BLineConstants.ROTATION_KI,
                BLineConstants.ROTATION_KD),
            new PIDController(
                BLineConstants.CROSS_TRACK_KP,
                BLineConstants.CROSS_TRACK_KI,
                BLineConstants.CROSS_TRACK_KD))
            .withDefaultShouldFlip()
            .withTRatioBasedTranslationHandoffs(true);

    m_autoChooser.setDefaultOption(
        "Do Nothing",
        Commands.none());

    m_autoChooser.addOption(
        "BLine Test",
        followBLinePath("test"));

    SmartDashboard.putData(
        "Autonomous",
        m_autoChooser);
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }


  public void setPerspective() {

    Optional<Alliance> ally =
        DriverStation.getAlliance();

    if (ally.isPresent()) {

      if (ally.get() == Alliance.Red) {

        m_driveTrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain
                .kRedAlliancePerspectiveRotation);
        m_hubPos =
            Constants.HubConstants.RED_HUB_POS;
      }

      if (ally.get() == Alliance.Blue) {

        m_driveTrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain
                .kBlueAlliancePerspectiveRotation);
        m_hubPos =
            Constants.HubConstants.BLUE_HUB_POS;
      }
    }
  }

  public void configure_bindings(
      java.util.function.DoubleSupplier driveRequest,
      java.util.function.DoubleSupplier strafeRequest,
      java.util.function.DoubleSupplier rotateRequest) {

    m_driveRequest = driveRequest;
    m_strafeRequest = strafeRequest;
    m_rotateRequest = rotateRequest;
  }

  public void setDriveState(
      DriveStates driveState) {

    m_driveState = driveState;
  }

  public Pose2d getPose() {

    return m_driveTrain
        .getState()
        .Pose;
  }

  public void resetPose(Pose2d pose) {
    m_driveTrain.resetPose(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_driveTrain
        .getState()
        .Speeds;
  }

  public void driveRobotRelative(
      ChassisSpeeds speeds) {

    m_driveTrain.setControl(
        m_blineDriveRequest
            .withSpeeds(speeds));
  }


  public Command followBLinePath(
      String pathName) {

    return m_blinePathBuilder.build(
        new Path(pathName));
  }

  public Command getAutonomousCommand() {

    return m_autoChooser.getSelected();
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {

    ChassisSpeeds robotRelativeSpeeds =
        m_driveTrain
            .getState()
            .Speeds;

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds,
        getPose().getRotation());
  }

  @Override
  public void periodic() {
    updateTurretLimelightPose();

    LimelightHelpers.PoseEstimate limelightEstimate =
        getFilteredPoseEstimate();

    if (limelightEstimate != null
        && limelightEstimate.tagCount > 0) {

      if (!DriverStation.isDisabled()) {

        m_driveTrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(
                Constants.LimelightConstants.VISION_STD_DEV_X,
                Constants.LimelightConstants.VISION_STD_DEV_Y,
                Constants.LimelightConstants.VISION_STD_DEV_THETA));

        m_driveTrain.addVisionMeasurement(
            limelightEstimate.pose.toPose2d(),
            Utils.fpgaToCurrentTime(
                limelightEstimate.timestampSeconds));

      } else {

        m_driveTrain.setVisionMeasurementStdDevs(
            VecBuilder.fill(
                0.1,
                0.1,
                0.1));

        m_driveTrain.addVisionMeasurement(
            limelightEstimate.pose.toPose2d(),
            Utils.fpgaToCurrentTime(
                limelightEstimate.timestampSeconds));
      }

      Logger.recordOutput(
          getName() + "/LimeLight Pose",
          limelightEstimate.pose);
    }

    if (limelightEstimate != null) {

      Logger.recordOutput(
          getName() + "/TagCount",
          limelightEstimate.tagCount);
    }
  }


  private void updateTurretLimelightPose() {

    double turretAngle =
        frc.robot.subsystems.shooter.ShooterSubsystem
            .getInstance()
            .getTurretAngleRelativeRobot();

    double cameraForward =
        Constants.LimelightConstants
            .CAMERA_FORWARD_METERS;
    double cameraLeft =
        Constants.LimelightConstants
            .CAMERA_LEFT_METERS;

    double cos =
        Math.cos(turretAngle);
    double sin =
        Math.sin(turretAngle);
    double rotatedForward =
        cameraForward * cos
            - cameraLeft * sin;
    double rotatedLeft =
        cameraForward * sin
            + cameraLeft * cos;
    double robotSpaceX =
        Constants.LimelightConstants
            .TURRET_PIVOT_FORWARD_METERS
            + rotatedForward;
    double robotSpaceY =
        Constants.LimelightConstants
            .TURRET_PIVOT_LEFT_METERS
            + rotatedLeft;
    double robotSpaceZ =
        Constants.LimelightConstants
            .TURRET_PIVOT_HEIGHT_METERS
            + Constants.LimelightConstants
                .CAMERA_HEIGHT_METERS;
    double cameraYaw =
        Constants.LimelightConstants
            .CAMERA_YAW_AT_ZERO_DEG
            + Math.toDegrees(turretAngle);

    LimelightHelpers.setCameraPose_RobotSpace(
        Constants.LimelightConstants.LIMELIGHT_NAME,
        robotSpaceX,
        robotSpaceY,
        robotSpaceZ,
        Constants.LimelightConstants.CAMERA_ROLL_DEG,
        Constants.LimelightConstants.CAMERA_PITCH_DEG,
        cameraYaw);
  }

  private LimelightHelpers.PoseEstimate
      getFilteredPoseEstimate() {

    LimelightHelpers.PoseEstimate pose_estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(
            Constants.LimelightConstants.LIMELIGHT_NAME);

    if (pose_estimate == null) {
      return null;
    }

    if (Math.abs(
            m_driveTrain
                .getState()
                .Speeds
                .omegaRadiansPerSecond)
        > Math.toRadians(
            Constants.LimelightConstants
                .MAX_VISION_ANGULAR_VELOCITY_DEG_PER_SEC)) {

      return null;
    }

    if (pose_estimate.tagCount == 0) {
      return null;
    }

    if (Double.isNaN(
            pose_estimate.pose.getX())
        || Double.isNaN(
            pose_estimate.pose.getY())
        || Double.isNaN(
            pose_estimate.pose
                .toPose2d()
                .getRotation()
                .getDegrees())) {

      return null;
    }

    if (pose_estimate.tagCount == 1
        && pose_estimate.rawFiducials.length == 1) {

      RawFiducial tag =
          pose_estimate.rawFiducials[0];

      if (tag.ambiguity
          > Constants.DriveConstants
              .SINGLE_TAG_AMBIGUITY_CUTOFF) {

        return null;
      }

      if (tag.distToCamera
          > Constants.DriveConstants
              .SINGLE_TAG_DISTANCE_CUTOFF) {

        return null;
      }
    }


    return pose_estimate;
  }
}