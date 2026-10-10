// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.BLine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends StateMachine {
  public enum DriveStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        getInstance()
            .m_driveTrain
            .setControl(
                getInstance().m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }

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
            Constants.Drive.MAX_ANGULAR_RATE.times(-getInstance().m_rotateRequest.getAsDouble());

        getInstance()
            .m_driveTrain
            .setControl(
                getInstance()
                    .m_drive
                    .withVelocityX(
                        Constants.Drive.MAX_SPEED.times(
                            -getInstance().m_driveRequest.getAsDouble()
                                * Math.abs(getInstance().m_driveRequest.getAsDouble())))
                    .withVelocityY(
                        Constants.Drive.MAX_SPEED.times(
                            -getInstance().m_strafeRequest.getAsDouble()
                                * Math.abs(getInstance().m_strafeRequest.getAsDouble())))
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

  private Rotation2d requestedPreMatch = null;
  private boolean shouldMirror; // midline
  private boolean shouldFlip; // alliance

  private CommandSwerveDrivetrain m_driveTrain;
  private SwerveRequest.FieldCentric m_drive;
  protected final Thread m_limelight_thread;
  private static volatile Map<String, Double> m_limelightHeartbeat;

  private final SwerveRequest.ApplyRobotSpeeds m_blineDriveRequest =
      new SwerveRequest.ApplyRobotSpeeds();
  private final FollowPath.Builder m_blinePathBuilder;
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  public DriveSubsystem() {

    super(DriveStates.REST);

    m_driveTrain = TunerConstants.createDrivetrain();
    setPerspective();

    m_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MAX_SPEED.times(Constants.Drive.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1))
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            BLine.MAX_VELOCITY_MPS,
            BLine.MAX_ACCELERATION_MPS2,
            BLine.MAX_ANGULAR_VELOCITY_DEG_PER_SEC,
            BLine.MAX_ANGULAR_ACCELERATION_DEG_PER_SEC2,
            BLine.END_TRANSLATION_TOLERANCE_METERS,
            BLine.END_ROTATION_TOLERANCE_DEG,
            BLine.INTERMEDIATE_HANDOFF_RADIUS_METERS));

    m_blinePathBuilder =
        new FollowPath.Builder(
                this,
                this::getPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PIDController(BLine.TRANSLATION_KP, BLine.TRANSLATION_KI, BLine.TRANSLATION_KD),
                new PIDController(BLine.ROTATION_KP, BLine.ROTATION_KI, BLine.ROTATION_KD),
                new PIDController(BLine.CROSS_TRACK_KP, BLine.CROSS_TRACK_KI, BLine.CROSS_TRACK_KD))
            .withDefaultShouldFlip()
            .withTRatioBasedTranslationHandoffs(true);

    m_autoChooser.setDefaultOption("Do Nothing", "nothing");

    m_autoChooser.addOption("BLine Test", "test");

    SmartDashboard.putData("Autonomous", m_autoChooser);

    SmartDashboard.putBoolean("shouldFlip", shouldFlip);
    SmartDashboard.putBoolean("shouldMirror", shouldMirror);

    m_limelightHeartbeat = new HashMap<>();
    m_limelightHeartbeat.put(Constants.Limelight.LIMELIGHT_NAME, 0.0);

    m_limelight_thread = new Thread(this::limelightThread);
    m_limelight_thread.setDaemon(true);
    m_limelight_thread.start();
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  public void setPerspective() {

    Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        m_driveTrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      }

      if (ally.get() == Alliance.Blue) {
        m_driveTrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
    }
  }

  public void configureBindings(
      DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    m_driveRequest = driveRequest;
    m_strafeRequest = strafeRequest;
    m_rotateRequest = rotateRequest;
  }

  public void setState(DriveStates driveState) {

    m_driveState = driveState;
  }

  public Pose2d getPose() {

    return m_driveTrain.getState().Pose;
  }

  public void resetPose(Pose2d pose) {
    m_driveTrain.resetPose(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_driveTrain.getState().Speeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    m_driveTrain.setControl(m_blineDriveRequest.withSpeeds(speeds));
  }

  public boolean underTower() {
    return (Constants.Field.BLUE_TOWER.contains(getPose().getTranslation())
        || Constants.Field.RED_TOWER.contains(getPose().getTranslation()));
  }

  public Command followBLinePath(Path path) {

    return m_blinePathBuilder.build(path);
  }

  public Command getAutonomousCommand() {
    Path path = new Path(m_autoChooser.getSelected());

    if (shouldFlip) {
      path.flip();
    }
    if (shouldMirror) {
      path.mirror();
    }

    Command orientModules =
        Commands.runOnce(
            () -> this.setModuleOrientations(path.getInitialModuleDirection(this::getPose)));

    Command auto =
        Commands.sequence(
            orientModules,
            Commands.waitUntil(this::modulesAtRequestedOrientation),
            followBLinePath(path));

    return auto;
  }

  private void setModuleOrientations(Rotation2d rot) {
    SwerveRequest.PointWheelsAt pointWheelsRequest = new SwerveRequest.PointWheelsAt();
    requestedPreMatch = rot;
    getInstance().m_driveTrain.setControl(pointWheelsRequest.withModuleDirection(rot));
  }

  private boolean modulesAtRequestedOrientation() {

    SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getInstance().m_driveTrain.getModules();

    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : modules) {
      SwerveModulePosition pos = module.getPosition(true);

      if (Math.abs(pos.angle.getDegrees() - getInstance().requestedPreMatch.getDegrees())
          > Constants.BLine.PREMATCH_MODULE_TOLERANCE.in(Degrees)) {
        return false;
      }
    }
    return true;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {

    ChassisSpeeds robotRelativeSpeeds = m_driveTrain.getState().Speeds;

    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getPose().getRotation());
  }

  private void limelightThread() {

    while (true) {
      updateTurretLimelightPose();

      LimelightHelpers.PoseEstimate limelightEstimate = getFilteredPoseEstimate();

      if (limelightEstimate != null && limelightEstimate.tagCount > 0) {

        if (!DriverStation.isDisabled()) {

          m_driveTrain.setVisionMeasurementStdDevs(
              VecBuilder.fill(
                  Constants.Limelight.VISION_STD_DEV_X,
                  Constants.Limelight.VISION_STD_DEV_Y,
                  Constants.Limelight.VISION_STD_DEV_THETA));

          m_driveTrain.addVisionMeasurement(
              limelightEstimate.pose.toPose2d(),
              Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));

        } else {

          m_driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));

          m_driveTrain.addVisionMeasurement(
              limelightEstimate.pose.toPose2d(),
              Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));
        }

        Logger.recordOutput(getName() + "/LimeLight Pose", limelightEstimate.pose);
      }

      try {
        Thread.sleep(15);
      } catch (InterruptedException e) {
      }
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("DriveSubsystem/Pose", m_driveTrain.getState().Pose);
    Logger.recordOutput("DriveSubsystem/State", getInstance().m_driveState);
  }

  private void updateTurretLimelightPose() {

    double turretAngle =
        frc.robot.subsystems.shooter.ShooterSubsystem.getInstance().getTurretRotation();

    turretAngle *= 2 * Math.PI;

    double cameraForward = Constants.Limelight.CAMERA_FORWARD.in(Meters);
    double cameraLeft = Constants.Limelight.CAMERA_LEFT.in(Meters);

    double cos = Math.cos(turretAngle);
    double sin = Math.sin(turretAngle);
    double rotatedForward = cameraForward * cos - cameraLeft * sin;
    double rotatedLeft = cameraForward * sin + cameraLeft * cos;
    double robotSpaceX = Constants.Limelight.TURRET_PIVOT_FORWARD_METERS + rotatedForward;
    double robotSpaceY = Constants.Limelight.TURRET_PIVOT_LEFT_METERS + rotatedLeft;
    double robotSpaceZ =
        Constants.Limelight.TURRET_PIVOT_HEIGHT_METERS
            + Constants.Limelight.CAMERA_HEIGHT.in(Meters);
    double cameraYaw =
        Constants.Limelight.CAMERA_YAW_AT_ZERO.in(Degrees) + Math.toDegrees(turretAngle);

    LimelightHelpers.setCameraPose_RobotSpace(
        Constants.Limelight.LIMELIGHT_NAME,
        robotSpaceX,
        robotSpaceY,
        robotSpaceZ,
        Constants.Limelight.CAMERA_ROLL.in(Degrees),
        Constants.Limelight.CAMERA_PITCH.in(Degrees),
        cameraYaw);
  }

  private LimelightHelpers.PoseEstimate getFilteredPoseEstimate() {
    LimelightHelpers.PoseEstimate pose_estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Limelight.LIMELIGHT_NAME);

    String limelight = Constants.Limelight.LIMELIGHT_NAME;
    double hb = LimelightHelpers.getHeartbeat(limelight);
    Double savedHb = m_limelightHeartbeat.get(limelight);

    if (savedHb == null || savedHb == hb) {
      return null;
    } else {
      m_limelightHeartbeat.put(limelight, hb);
    }

    if (pose_estimate == null) {
      return null;
    }

    if (Math.abs(m_driveTrain.getState().Speeds.omegaRadiansPerSecond)
        > Constants.Limelight.MAX_VISION_ANGULAR_VELOCITY_PER_SEC.in(Radians)) {

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

    if (pose_estimate.tagCount == 1 && pose_estimate.rawFiducials.length == 1) {

      RawFiducial tag = pose_estimate.rawFiducials[0];

      if (tag.ambiguity > Constants.Drive.SINGLE_TAG_AMBIGUITY_CUTOFF) {

        return null;
      }

      if (tag.distToCamera > Constants.Drive.SINGLE_TAG_DISTANCE_CUTOFF) {

        return null;
      }
    }

    return pose_estimate;
  }
}
