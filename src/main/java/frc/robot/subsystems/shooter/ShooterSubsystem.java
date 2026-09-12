// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends StateMachine {

  public enum ShooterStates implements SystemState {
    OFF {
      @Override
      public void initialize() {
        //
        // getInstance().m_shooterLeader.setControl(getInstance().m_velocityVoltage.withVelocity(0));
        //   getInstance().m_hoodMotor.setControl(getInstance().m_positionVoltage.withPosition(0));
        // getInstance.updateTurretEncoder();
        // getInstance().m_turretMotor.setControl(getInstance().m_positionVoltage.withPosition(0))
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ON {
      @Override
      public void execute() {
        Translation2d robotPose =
            getInstance()
                .transformByTangentialRotationSpeed(
                    getInstance().getFuturePose(Constants.ShooterConstants.HANG_TIME));
        Translation2d targetDiff = getInstance().getTarget().minus(robotPose);
        double distance = targetDiff.getNorm();
        double x_vel =
            getVelocityXStationary(
                distance,
                Constants.ShooterConstants.HUB_HEIGHT,
                Constants.ShooterConstants.MAX_BALL_Y_POS.getAsDouble());
        double y_vel =
            getVelocityYStationary(Constants.ShooterConstants.MAX_BALL_Y_POS.getAsDouble());
        Logger.recordOutput(
            "ShooterSubsystem/HoodAngle",
            getInstance().getHoodPos(x_vel, y_vel) / (2 * Math.PI) * 360);
        Logger.recordOutput(
            "ShooterSubsystem/FlywheelSpeed", getInstance().getShooterSpeed(x_vel, y_vel));
        Logger.recordOutput(
            "ShooterSubsystem/FuturePoseHangTime", new Pose2d(robotPose, new Rotation2d(0)));
        Logger.recordOutput(
            "ShooterSubsystem/TurretPos",
            new Pose2d(
                robotPose,
                new Rotation2d(
                    getInstance()
                        .getTurretPos(
                            getInstance().getTarget(),
                            new Pose2d(
                                robotPose,
                                DriveSubsystem.getInstance().getPose().getRotation())))));
        Logger.recordOutput(
            "ShooterSubsystem/AggregateAimPoint",
            new Pose2d(
                robotPose,
                new Rotation2d(
                    (getInstance()
                            .getTurretPos(
                                getInstance().getTarget(),
                                new Pose2d(
                                    robotPose,
                                    DriveSubsystem.getInstance().getPose().getRotation()))
                        - (getInstance().m_turretMotor.getPosition().getValueAsDouble()
                            / Constants.ShooterConstants.MOTOR_TURRET_GEAR_RATIO)))));

        /*  getInstance().setTurretPos(getInstance().getTurretPos(getInstance().getTarget(),new Pose2d(robotPose,DriveSubsystem.getInstance().getPose().getRotation()))))););
            getInstance().setHoodPos(getInstance().getHoodPos(x_vel, y_vel));
            getInstance().setShooterSpeed(getInstance().getShooterSpeed(x_vel, y_vel));
        */
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static ShooterSubsystem s_shooterInstance;
  private ShooterStates m_selectedState;
  private TalonFX m_shooterLeader;
  private TalonFX m_shooterFollower;
  private TalonFX m_hoodMotor;
  private CANcoder m_encoderOne;
  private CANcoder m_encoderTwo;
  private VelocityVoltage m_velocityVoltage;
  private PositionVoltage m_positionVoltage;
  private TalonFXConfiguration m_shooterConfig;
  private TalonFXConfiguration m_hoodConfig;
  private TalonFX m_turretMotor;
  private TalonFXConfiguration m_turretConfig;
  private boolean m_blueAlliance;

  public ShooterSubsystem() {
    super(ShooterStates.OFF);
    setState(ShooterStates.OFF);
    m_shooterLeader = new TalonFX(Constants.ShooterConstants.SHOOTER_LEADER_ID);
    m_shooterFollower = new TalonFX(Constants.ShooterConstants.SHOOTER_FOLLOWER_ID);
    m_hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID);
    m_turretMotor = new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_ID);
    m_encoderOne = new CANcoder(Constants.ShooterConstants.ENCODER_ONE_ID);
    m_encoderOne = new CANcoder(Constants.ShooterConstants.ENCODER_TWO_ID);
    m_velocityVoltage = new VelocityVoltage(0);
    m_positionVoltage = new PositionVoltage(0);
    m_shooterFollower.setControl(
        new Follower(m_shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    m_shooterConfig = new TalonFXConfiguration();
    m_shooterConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_hoodConfig = new TalonFXConfiguration();
    m_hoodConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ShooterConstants.MOTOR_HOOD_GEAR_RATIO
            * Constants.ShooterConstants.HOOD_MAX_ANGLE;
    m_hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    m_hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_turretConfig = new TalonFXConfiguration(); // TODO SET PID SV VALUES FOR ALL SUBSYSTEMS
    m_turretConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 23.0;
    m_turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -23.0;
    m_turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_shooterLeader.getConfigurator().apply(m_shooterConfig);
    m_shooterFollower.getConfigurator().apply(m_shooterConfig);
    m_hoodMotor.getConfigurator().apply(m_hoodConfig);
    m_turretMotor.getConfigurator().apply(m_turretConfig);
    // updateTurretEncoder();
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }
    return s_shooterInstance;
  }

  public void setState(ShooterStates state) {
    m_selectedState = state;
  }

  private boolean inAZ() {
    if (m_blueAlliance) {
      return DriveSubsystem.getInstance().getPose().getX() < Constants.FieldConstants.NZ_BLUE_X;
    } else {
      return DriveSubsystem.getInstance().getPose().getX() > Constants.FieldConstants.NZ_RED_X;
    }
  }

  private boolean inNZ() {
    return DriveSubsystem.getInstance().getPose().getX() > Constants.FieldConstants.NZ_BLUE_X
        && DriveSubsystem.getInstance().getPose().getX() < Constants.FieldConstants.NZ_RED_X;
  }

  private Translation2d getTarget() {
    if (m_blueAlliance) {
      if (inAZ()) {
        return Constants.FieldConstants.BLUE_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY()
            < Constants.FieldConstants.NZ_MID_LINE_Y) {
          return Constants.FieldConstants.BLUE_RIGHT_BUMP;
        }
        return Constants.FieldConstants.BLUE_LEFT_BUMP;
      }
    } else {
      if (inAZ()) {
        return Constants.FieldConstants.RED_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY()
            < Constants.FieldConstants.NZ_MID_LINE_Y) {
          return Constants.FieldConstants.RED_LEFT_BUMP;
        }
        return Constants.FieldConstants.RED_RIGHT_BUMP;
      }
    }
  }

  private double getTurretPos(Translation2d target, Pose2d robotPose) {
    if (target == null) {
      return 0;
    }
    double xOffset = target.getX() - robotPose.getX();
    double yOffset = target.getY() - robotPose.getY();
    double angleToTarget = robotPose.getRotation().getRadians() - Math.atan2(yOffset, xOffset);
    double turretDesired =
        -angleToTarget
            - (m_turretMotor.getPosition().getValueAsDouble()
                / Constants.ShooterConstants.MOTOR_TURRET_GEAR_RATIO);
    if (turretDesired < -Math.PI) {
      turretDesired += 2 * Math.PI;
    } else if (turretDesired > Math.PI) {
      turretDesired -= 2 * Math.PI;
    }
    return turretDesired;
  }

  private void setTurretPos(double desiredPos) {
    m_turretMotor.setControl(
        m_positionVoltage.withPosition(
            desiredPos / (2 * Math.PI) * Constants.ShooterConstants.MOTOR_TURRET_GEAR_RATIO));
  }

  private double getHoodPos(double x_vel, double y_vel) {
    if (robotCrossTrench()) {
      return 0;
    }
    double hoodAngle = (Math.PI / 2) - Math.atan2(y_vel, x_vel);
    hoodAngle =
        MathUtil.clamp(hoodAngle, 0, Constants.ShooterConstants.HOOD_MAX_ANGLE * 2 * Math.PI);
    return hoodAngle;
  }

  private void setHoodPos(double hoodPos) {
    m_hoodMotor.setControl(
        m_positionVoltage.withPosition(
            hoodPos / (2 * Math.PI) * Constants.ShooterConstants.MOTOR_HOOD_GEAR_RATIO));
  }

  private double getShooterSpeed(double x_vel, double y_vel) {
    double shootSpeed = Math.hypot(x_vel, y_vel);
    double desiredRPS = (shootSpeed * 4 / 3) / (Inches.of(4).in(Meters) * Math.PI);
    double finalRPS = desiredRPS * Constants.ShooterConstants.MOTOR_SHOOTER_GEAR_RATIO;
    return finalRPS;
  }

  private void setShooterSpeed(double speed) {
    m_shooterLeader.setControl(m_velocityVoltage.withVelocity(speed));
  }

  private static double getVelocityXStationary(
      double distance, double targetHeight, double maxBallYPos) {
    double y_max = maxBallYPos;
    double y_end = targetHeight;
    double g = Constants.FieldConstants.GRAVITY_VALUE;

    double x_vel =
        distance * (Math.sqrt(g)) / (Math.sqrt(2 * y_max) + Math.sqrt(2 * (y_max - y_end)));
    return x_vel;
  }

  private static double getVelocityYStationary(double maxBallYPos) {
    double y_max = maxBallYPos;
    double g = Constants.FieldConstants.GRAVITY_VALUE;

    double y_vel = Math.sqrt(y_max * 2 * g);
    return y_vel;
  }

  public boolean robotCrossTrench() {
    Translation2d a = DriveSubsystem.getInstance().getTranslation2d();
    Translation2d b = getFuturePose(Constants.ShooterConstants.HOOD_COLLISION_TIME);
    Translation2d c;
    Translation2d d;
    if (b.getX() > Constants.FieldConstants.NZ_MID_LINE_X) {
      if (b.getY() > Constants.FieldConstants.NZ_MID_LINE_Y) {
        c = Constants.FieldConstants.RED_LEFT_TRENCH_P1;
        d = Constants.FieldConstants.RED_LEFT_TRENCH_P2;
      } else {
        c = Constants.FieldConstants.RED_RIGHT_TRENCH_P1;
        d = Constants.FieldConstants.RED_RIGHT_TRENCH_P2;
      }
    } else {
      if (b.getY() > Constants.FieldConstants.NZ_MID_LINE_Y) {
        c = Constants.FieldConstants.BLUE_LEFT_TRENCH_P1;
        d = Constants.FieldConstants.BLUE_LEFT_TRENCH_P2;
      } else {
        c = Constants.FieldConstants.BLUE_RIGHT_TRENCH_P1;
        d = Constants.FieldConstants.BLUE_RIGHT_TRENCH_P2;
      }
    }
    /*Basically, form a line segment the length of the robot, and see if it intersects the trench
     * as well as checking current vs future pose to see if
     * robot will cross trench in forseeable future(HOOD_COLLISION_TIME secondsto be precise)
     */
    Translation2d toEdgeOfRobot = new Translation2d(Constants.ShooterConstants.CENTER_TO_EDGE, 0);
    boolean underTrench =
        (segmentsIntersect(a, b, c, d)
            || segmentsIntersect(a.minus(toEdgeOfRobot), a.plus(toEdgeOfRobot), c, d));
    return underTrench;
  }

  private double crossProductOrient(Translation2d a, Translation2d b, Translation2d c) {
    return (b.getX() - a.getX()) * (c.getY() - a.getY())
        - (b.getY() - a.getY()) * (c.getX() - a.getX());
  }

  /*Check all line segments of the 2 hubs and see if it intersects the robot and target line segment */
  public boolean canSeeTarget() {
    Translation2d a =
        DriveSubsystem.getInstance()
            .getPose()
            .transformBy(
                new Transform2d(
                    Constants.ShooterConstants.SHOOTER_OFFSET_X,
                    Constants.ShooterConstants.SHOOTER_OFFSET_Y,
                    new Rotation2d(0)))
            .getTranslation();
    Translation2d b = getTarget();
    Translation2d blueBottomRight =
        new Translation2d(
            Constants.FieldConstants.BLUE_HUB_POS.getX() - Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.BLUE_HUB_POS.getY() - Constants.FieldConstants.HUB_WIDTH / 2);
    Translation2d blueBottomLeft =
        new Translation2d(
            Constants.FieldConstants.BLUE_HUB_POS.getX() - Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.BLUE_HUB_POS.getY() + Constants.FieldConstants.HUB_WIDTH / 2);
    Translation2d blueTopRight =
        new Translation2d(
            Constants.FieldConstants.BLUE_HUB_POS.getX() + Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.BLUE_HUB_POS.getY() - Constants.FieldConstants.HUB_WIDTH / 2);
    Translation2d blueTopLeft =
        new Translation2d(
            Constants.FieldConstants.BLUE_HUB_POS.getX() + Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.BLUE_HUB_POS.getY() + Constants.FieldConstants.HUB_WIDTH / 2);
    Translation2d redBottomRight =
        new Translation2d(
            Constants.FieldConstants.RED_HUB_POS.getX() - Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.RED_HUB_POS.getY() - Constants.FieldConstants.HUB_WIDTH / 2);
    Translation2d redBottomLeft =
        new Translation2d(
            Constants.FieldConstants.RED_HUB_POS.getX() - Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.RED_HUB_POS.getY() + Constants.FieldConstants.HUB_WIDTH / 2);
    Translation2d redTopRight =
        new Translation2d(
            Constants.FieldConstants.RED_HUB_POS.getX() - Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.RED_HUB_POS.getY() + Constants.FieldConstants.HUB_WIDTH / 2);
    Translation2d redTopLeft =
        new Translation2d(
            Constants.FieldConstants.RED_HUB_POS.getX() + Constants.FieldConstants.HUB_WIDTH / 2,
            Constants.FieldConstants.RED_HUB_POS.getY() + Constants.FieldConstants.HUB_WIDTH / 2);
    if (segmentsIntersect(a, b, blueBottomLeft, blueBottomRight)) return false;
    if (segmentsIntersect(a, b, blueTopLeft, blueTopRight)) return false;
    if (segmentsIntersect(a, b, blueBottomRight, blueTopRight)) return false;
    if (segmentsIntersect(a, b, blueBottomLeft, blueTopLeft)) return false;
    if (segmentsIntersect(a, b, redBottomLeft, redBottomRight)) return false;
    if (segmentsIntersect(a, b, redTopLeft, redTopRight)) return false;
    if (segmentsIntersect(a, b, redBottomRight, redTopRight)) return false;
    if (segmentsIntersect(a, b, redBottomLeft, redTopLeft)) return false;
    return true;
  }

  /*
  Check if two line segments intersect ---
  Used for checking if either hub is between robot and target(Check all 4 line segment sides of the hub)
  And for checking if robot is under the trench
  */
  private boolean segmentsIntersect(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double orient1 = crossProductOrient(a, b, c);
    double orient2 = crossProductOrient(a, b, d);
    double orient3 = crossProductOrient(c, d, a);
    double orient4 = crossProductOrient(c, d, b);
    return ((orient1 > 0 && orient2 < 0) || (orient1 < 0 && orient2 > 0))
        && ((orient3 > 0 && orient4 < 0) || (orient3 < 0 && orient4 > 0));
  }

  private void updateTurretEncoder() {
    StatusSignal<Angle> encoderOneSignal = m_encoderOne.getPosition();
    StatusSignal<Angle> encoderTwoSignal = m_encoderTwo.getPosition();
    BaseStatusSignal.refreshAll(encoderOneSignal, encoderTwoSignal);
    BaseStatusSignal.waitForAll(0.1, encoderOneSignal, encoderTwoSignal);
    double encoderOnePosition = encoderOneSignal.getValue().in(Rotations);
    double encoderTwoPosition = encoderTwoSignal.getValue().in(Rotations);
    double[] encoderOnePossible = new double[Constants.ShooterConstants.ENCODER_ONE_TEETH];
    double[] encoderTwoPossible = new double[Constants.ShooterConstants.ENCODER_TWO_TEETH];

    for (int i = 0; i < Constants.ShooterConstants.ENCODER_ONE_TEETH; i++) {
      encoderOnePossible[i] =
          (i + encoderOnePosition)
              * ((double) Constants.ShooterConstants.ENCODER_ONE_TEETH
                  / Constants.ShooterConstants.TURRET_GEAR_TEETH);
    }
    for (int i = 0; i < Constants.ShooterConstants.ENCODER_TWO_TEETH; i++) {
      encoderTwoPossible[i] =
          (i + encoderTwoPosition)
              * ((double) Constants.ShooterConstants.ENCODER_TWO_TEETH
                  / Constants.ShooterConstants.TURRET_GEAR_TEETH);
    }

    double matchingValue = 0;
    outerLoop:
    for (double eOnePossible : encoderOnePossible) {
      for (double eTwoPossible : encoderTwoPossible) {
        if (Math.abs(eTwoPossible - eOnePossible) < Constants.ShooterConstants.CRT_EPSILON) {
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

  public double getTurretRotation() {
    return m_turretMotor.getPosition().getValueAsDouble()
        / Constants.ShooterConstants.MOTOR_TURRET_GEAR_RATIO;
  }

  private Translation2d getFuturePose(double time) {
    Pose2d currentPose = DriveSubsystem.getInstance().getPose();
    Translation2d futurePos =
        currentPose
            .getTranslation()
            .plus(
                new Translation2d(
                        DriveSubsystem.getInstance().getSpeeds().vxMetersPerSecond,
                        DriveSubsystem.getInstance().getSpeeds().vyMetersPerSecond)
                    .times(time));
    return futurePos;
  }

  /*The turret could be rotating with the robot,
  as such we want to transform out future pose
  by the tangential velocity and direction in order to have accurate SOTM */
  private Translation2d transformByTangentialRotationSpeed(Translation2d currentPos) {
    double linearTangentSpeed =
        MathUtil.applyDeadband(
                DriveSubsystem.getInstance().getSpeeds().omegaRadiansPerSecond,
                Constants.DriveConstants.ROTATION_DEADBAND)
            * Constants.ShooterConstants.SHOOTER_OFFSET_RADIUS;
    Translation2d transformationVector =
        new Translation2d(
            linearTangentSpeed * Constants.ShooterConstants.HANG_TIME,
            DriveSubsystem.getInstance()
                .getPose()
                .getRotation()
                .minus(new Rotation2d(Radians.of(-Math.PI / 2))));
    return currentPos.plus(transformationVector);
  }

  @Override
  public void periodic() {
    m_blueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Logger.recordOutput("ShooterSubsystem/InNZ", inNZ());
    Logger.recordOutput("ShooterSubsystem/InAZ", inAZ());
    Logger.recordOutput("ShooterSubsystem/Target", getTarget());
    Logger.recordOutput(
        "ShooterSubsystem/FuturePoseHoodCollisionTime",
        new Pose2d(
            getFuturePose(Constants.ShooterConstants.HOOD_COLLISION_TIME),
            DriveSubsystem.getInstance().getPose().getRotation()));
    Logger.recordOutput("ShooterSubsystem/UnderTrench", robotCrossTrench());
  }
}
