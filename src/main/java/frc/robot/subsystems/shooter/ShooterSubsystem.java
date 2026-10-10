// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
        getInstance().m_readytoShoot = false;

        if (!RobotBase.isSimulation()) {
          Thread turretUpdateThread = new Thread(() -> getInstance().updateTurretEncoder());
          turretUpdateThread.start();
        }

        getInstance().m_shooterLeader.setControl(getInstance().m_shooterRequest.withVelocity(0));
        getInstance().m_hoodMotor.setControl(getInstance().m_hoodRequest.withPosition(0));
        getInstance().m_turretMotor.setControl(getInstance().m_turretRequest.withPosition(0));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ZERO {
      Timer m_zeroTimer = new Timer();

      @Override
      public void initialize() {
        getInstance().m_hoodMotor.setVoltage(Constants.Shooter.ZERO_VOLTAGE.in(Volts));
        m_zeroTimer.reset();
        m_zeroTimer.start();
      }

      @Override
      public void execute() {
        if (m_zeroTimer.hasElapsed(Constants.Shooter.ZEROING_DELAY)
            && getInstance().m_hoodMotor.getTorqueCurrent().getValueAsDouble()
                >= Constants.Shooter.ZERO_THRESHOLD.in(Amps)) {
          getInstance().m_hoodMotor.setPosition(0.0);
          getInstance().setFinishedZero(true);
          getInstance().setState(ON);
        }
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
                    getInstance().getFuturePose(Constants.Shooter.HANG_TIME));
        Translation3d target = getInstance().getTarget();
        Translation2d targetDiff = target.toTranslation2d().minus(robotPose);
        double distance = targetDiff.getNorm();
        double x_vel =
            getVelocityXStationary(
                distance, target.getZ(), Constants.Shooter.MAX_BALL_Y_POS.getAsDouble());
        double y_vel = getVelocityYStationary(Constants.Shooter.MAX_BALL_Y_POS.getAsDouble());

        getInstance().m_readytoShoot =
            getInstance()
                .getTurretPosition()
                .isNear(
                    getInstance()
                        .getDesiredTurretPos(
                            target.toTranslation2d(),
                            new Pose2d(
                                robotPose, DriveSubsystem.getInstance().getPose().getRotation())),
                    Constants.Shooter.TURRET_DEADBAND.in(Radians));

        Logger.recordOutput("ShooterSubsystem/State", getInstance().getState().toString());
        Logger.recordOutput(
            "ShooterSubsystem/HoodAngle", getInstance().getHoodPos(x_vel, y_vel).in(Degrees));
        Logger.recordOutput(
            "ShooterSubsystem/FlywheelSpeed", getInstance().getShooterSpeed(x_vel, y_vel));
        Logger.recordOutput(
            "ShooterSubsystem/FuturePoseHangTime", new Pose2d(robotPose, new Rotation2d(0)));
        Logger.recordOutput(
            "ShooterSubsystem/TurretPos",
            new Pose2d(
                robotPose,
                Rotation2d.fromDegrees(
                    getInstance()
                        .getDesiredTurretPos(
                            target.toTranslation2d(),
                            new Pose2d(
                                robotPose, DriveSubsystem.getInstance().getPose().getRotation()))
                        .in(Degrees))));
        Logger.recordOutput(
            "ShooterSubsystem/AggregateAimPoint",
            new Pose2d(
                DriveSubsystem.getInstance().getPose().getTranslation(),
                DriveSubsystem.getInstance()
                    .getPose()
                    .getRotation()
                    .plus(
                        Rotation2d.fromDegrees(
                                getInstance()
                                    .getDesiredTurretPos(
                                        target.toTranslation2d(),
                                        new Pose2d(
                                            robotPose,
                                            DriveSubsystem.getInstance().getPose().getRotation()))
                                    .in(Degrees))
                            .minus(
                                Rotation2d.fromDegrees(
                                    getInstance().getTurretPosition().in(Degrees))))));

        getInstance()
            .setTurretPos(
                getInstance()
                    .getDesiredTurretPos(
                        target.toTranslation2d(),
                        new Pose2d(
                            robotPose, DriveSubsystem.getInstance().getPose().getRotation())));
        getInstance().setHoodPos(getInstance().getHoodPos(x_vel, y_vel));
        getInstance().setShooterSpeed(getInstance().getShooterSpeed(x_vel, y_vel));
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

  private VelocityVoltage m_shooterRequest;
  private MotionMagicVoltage m_hoodRequest;
  private MotionMagicVoltage m_turretRequest;

  private TalonFX m_turretMotor;

  private boolean m_blueAlliance;
  private boolean m_readytoShoot;
  private boolean m_finishedZero;

  public ShooterSubsystem() {
    super(ShooterStates.OFF);
    m_finishedZero = false;
    setState(ShooterStates.OFF);
    m_shooterLeader = new TalonFX(Constants.Shooter.SHOOTER_LEADER_ID);
    m_shooterFollower = new TalonFX(Constants.Shooter.SHOOTER_FOLLOWER_ID);
    m_hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
    m_turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID);
    m_encoderOne = new CANcoder(Constants.Shooter.ENCODER_ONE_ID);
    m_encoderTwo = new CANcoder(Constants.Shooter.ENCODER_TWO_ID);

    m_shooterRequest = new VelocityVoltage(0);
    m_hoodRequest = new MotionMagicVoltage(0);
    m_turretRequest = new MotionMagicVoltage(0);

    m_shooterFollower.setControl(
        new Follower(m_shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    TalonFXConfiguration turretConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(75).withKS(0.2197265625))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(46))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Rotations.of(0.25))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.of(-0.25)))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(2.5))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1.5)));

    CANcoderConfiguration encoderOneConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.99))
                    .withMagnetOffset(Rotations.of(-0.417236328125))
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

    CANcoderConfiguration encoderTwoConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.99))
                    .withMagnetOffset(Rotations.of(-0.396728515625))
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

    m_shooterLeader.getConfigurator().apply(shooterConfig);
    m_shooterFollower.getConfigurator().apply(shooterConfig);
    m_hoodMotor.getConfigurator().apply(hoodConfig);
    m_turretMotor.getConfigurator().apply(turretConfig);
    m_encoderOne.getConfigurator().apply(encoderOneConfig);
    m_encoderTwo.getConfigurator().apply(encoderTwoConfig);
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

  public boolean inAZ() {
    if (m_blueAlliance) {
      return DriveSubsystem.getInstance().getPose().getX() < Constants.Field.NZ_BLUE_X;
    } else {
      return DriveSubsystem.getInstance().getPose().getX() > Constants.Field.NZ_RED_X;
    }
  }

  public boolean inNZ() {
    return DriveSubsystem.getInstance().getPose().getX() > Constants.Field.NZ_BLUE_X
        && DriveSubsystem.getInstance().getPose().getX() < Constants.Field.NZ_RED_X;
  }

  private Translation3d getTarget() {
    if (m_blueAlliance) {
      if (inAZ()) {
        return Constants.Field.BLUE_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY() < Constants.Field.NZ_MID_LINE_Y) {
          return Constants.Field.BLUE_RIGHT_BUMP;
        }
        return Constants.Field.BLUE_LEFT_BUMP;
      }
    } else {
      if (inAZ()) {
        return Constants.Field.RED_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY() < Constants.Field.NZ_MID_LINE_Y) {
          return Constants.Field.RED_LEFT_BUMP;
        }
        return Constants.Field.RED_RIGHT_BUMP;
      }
    }
  }

  private Angle getDesiredTurretPos(Translation2d target, Pose2d robotPose) {
    if (target == null) {
      return Degrees.of(0);
    }
    double xOffset = target.getX() - robotPose.getX();
    double yOffset = target.getY() - robotPose.getY();
    Angle angleToTarget =
        Radians.of(robotPose.getRotation().getRadians() - Math.atan2(yOffset, xOffset));
    Angle turretDesired = angleToTarget.times(-1).minus(getTurretPosition());
    if (turretDesired.lt(Degrees.of(-90))) {
      turretDesired = turretDesired.plus(Rotations.of(1));
    } else if (turretDesired.gt(Degrees.of(90))) {
      turretDesired = turretDesired.minus(Rotations.of(1));
    }
    return turretDesired;
  }

  private void setTurretPos(Angle desiredPos) {
    m_turretMotor.setControl(m_turretRequest.withPosition(desiredPos));
  }

  private Angle getHoodPos(double x_vel, double y_vel) {
    if (robotCrossTrench()) {
      return Degrees.of(0);
    }
    Angle hoodAngle = Radians.of(((Math.PI / 2) - Math.atan2(y_vel, x_vel)));
    return hoodAngle;
  }

  private void setHoodPos(Angle hoodPos) {
    m_hoodMotor.setControl(m_hoodRequest.withPosition(hoodPos));
  }

  private double getShooterSpeed(double x_vel, double y_vel) {
    double shootSpeed = Math.hypot(x_vel, y_vel);
    double desiredRPS = (shootSpeed * 4 / 3) / (Inches.of(4).in(Meters) * Math.PI);
    ;
    return desiredRPS;
  }

  private void setShooterSpeed(double speed) {
    m_shooterLeader.setControl(m_shooterRequest.withVelocity(speed));
  }

  private static double getVelocityXStationary(
      double distance, double targetHeight, double maxBallYPos) {
    double y_max = maxBallYPos;
    double y_end = targetHeight;
    double g = Constants.Field.GRAVITY_VALUE;

    double x_vel =
        distance * (Math.sqrt(g)) / (Math.sqrt(2 * y_max) + Math.sqrt(2 * (y_max - y_end)));
    return x_vel;
  }

  private static double getVelocityYStationary(double maxBallYPos) {
    double y_max = maxBallYPos;
    double g = Constants.Field.GRAVITY_VALUE;

    double y_vel = Math.sqrt(y_max * 2 * g);
    return y_vel;
  }

  public boolean robotCrossTrench() {
    Translation2d a = DriveSubsystem.getInstance().getPose().getTranslation();
    Translation2d b = getFuturePose(Constants.Shooter.HOOD_COLLISION_TIME);
    Translation2d c;
    Translation2d d;
    if (b.getX() > Constants.Field.NZ_MID_LINE_X) {
      if (b.getY() > Constants.Field.NZ_MID_LINE_Y) {
        c = Constants.Field.RED_LEFT_TRENCH_P1;
        d = Constants.Field.RED_LEFT_TRENCH_P2;
      } else {
        c = Constants.Field.RED_RIGHT_TRENCH_P1;
        d = Constants.Field.RED_RIGHT_TRENCH_P2;
      }
    } else {
      if (b.getY() > Constants.Field.NZ_MID_LINE_Y) {
        c = Constants.Field.BLUE_LEFT_TRENCH_P1;
        d = Constants.Field.BLUE_LEFT_TRENCH_P2;
      } else {
        c = Constants.Field.BLUE_RIGHT_TRENCH_P1;
        d = Constants.Field.BLUE_RIGHT_TRENCH_P2;
      }
    }
    /*Basically, form a line segment the length of the robot, and see if it intersects the trench
     * as well as checking current vs future pose to see if
     * robot will cross trench in forseeable future(HOOD_COLLISION_TIME secondsto be precise)
     */
    Translation2d toEdgeOfRobot = new Translation2d(Constants.Shooter.CENTER_TO_EDGE.in(Meters), 0);
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
    if (inAZ()) {
      return true;
    }
    Translation2d a =
        DriveSubsystem.getInstance()
            .getPose()
            .transformBy(
                new Transform2d(
                    Constants.Shooter.SHOOTER_OFFSET_X,
                    Constants.Shooter.SHOOTER_OFFSET_Y,
                    new Rotation2d(0)))
            .getTranslation();
    Translation2d b = getTarget().toTranslation2d();
    Translation2d blueBottomRight =
        new Translation2d(
            Constants.Field.BLUE_HUB_POS.getX() - Constants.Field.HUB_WIDTH / 2,
            Constants.Field.BLUE_HUB_POS.getY() - Constants.Field.HUB_WIDTH / 2);
    Translation2d blueBottomLeft =
        new Translation2d(
            Constants.Field.BLUE_HUB_POS.getX() - Constants.Field.HUB_WIDTH / 2,
            Constants.Field.BLUE_HUB_POS.getY() + Constants.Field.HUB_WIDTH / 2);
    Translation2d blueTopRight =
        new Translation2d(
            Constants.Field.BLUE_HUB_POS.getX() + Constants.Field.HUB_WIDTH / 2,
            Constants.Field.BLUE_HUB_POS.getY() - Constants.Field.HUB_WIDTH / 2);
    Translation2d blueTopLeft =
        new Translation2d(
            Constants.Field.BLUE_HUB_POS.getX() + Constants.Field.HUB_WIDTH / 2,
            Constants.Field.BLUE_HUB_POS.getY() + Constants.Field.HUB_WIDTH / 2);
    Translation2d redBottomRight =
        new Translation2d(
            Constants.Field.RED_HUB_POS.getX() - Constants.Field.HUB_WIDTH / 2,
            Constants.Field.RED_HUB_POS.getY() - Constants.Field.HUB_WIDTH / 2);
    Translation2d redBottomLeft =
        new Translation2d(
            Constants.Field.RED_HUB_POS.getX() - Constants.Field.HUB_WIDTH / 2,
            Constants.Field.RED_HUB_POS.getY() + Constants.Field.HUB_WIDTH / 2);
    Translation2d redTopRight =
        new Translation2d(
            Constants.Field.RED_HUB_POS.getX() + Constants.Field.HUB_WIDTH / 2,
            Constants.Field.RED_HUB_POS.getY() - Constants.Field.HUB_WIDTH / 2);
    Translation2d redTopLeft =
        new Translation2d(
            Constants.Field.RED_HUB_POS.getX() + Constants.Field.HUB_WIDTH / 2,
            Constants.Field.RED_HUB_POS.getY() + Constants.Field.HUB_WIDTH / 2);
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
    double[] encoderOnePossible = new double[Constants.Shooter.ENCODER_ONE_TEETH];
    double[] encoderTwoPossible = new double[Constants.Shooter.ENCODER_TWO_TEETH];
    /*
     * Basically, the turret rotates from -0.5 to 0.5 rotations,
     * so based on this, as well as the period of alignment with the encoders (mod smth)
     * we want to check negative and positive domains of i in this case
     *
     */
    for (int i = -5; i < 10; i++) {
      encoderOnePossible[i] =
          (i + encoderOnePosition)
              * ((double) Constants.Shooter.ENCODER_ONE_TEETH
                  / Constants.Shooter.TURRET_GEAR_TEETH);
    }
    for (int i = 0; i < Constants.Shooter.ENCODER_TWO_TEETH; i++) {
      encoderTwoPossible[i] =
          (i + encoderTwoPosition)
              * ((double) Constants.Shooter.ENCODER_TWO_TEETH
                  / Constants.Shooter.TURRET_GEAR_TEETH);
    }

    double matchingValue = 0;
    boolean foundSolution = false;
    outerLoop:
    for (double eOnePossible : encoderOnePossible) {
      for (double eTwoPossible : encoderTwoPossible) {
        if (Math.abs(eTwoPossible - eOnePossible) < Constants.Shooter.CRT_EPSILON) {
          matchingValue = (eOnePossible + eTwoPossible) / 2;
          foundSolution = true;
          break outerLoop;
        }

        if (eTwoPossible > eOnePossible) {
          break;
        }
      }
    }

    if (foundSolution) {
      m_turretMotor.setPosition(matchingValue);
    }
  }

  public Angle getTurretPosition() {
    return m_turretMotor.getPosition().getValue();
  }

  private Translation2d getFuturePose(double time) {
    Pose2d currentPose = DriveSubsystem.getInstance().getPose();
    Translation2d futurePos =
        currentPose
            .getTranslation()
            .plus(
                new Translation2d(
                        DriveSubsystem.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond,
                        DriveSubsystem.getInstance().getFieldRelativeSpeeds().vyMetersPerSecond)
                    .times(time));
    return futurePos;
  }

  /*The turret could be rotating with the robot,
  as such we want to transform out future pose
  by the tangential velocity and direction in order to have accurate SOTM */
  private Translation2d transformByTangentialRotationSpeed(Translation2d currentPos) {
    double linearTangentSpeed =
        MathUtil.applyDeadband(
                DriveSubsystem.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond,
                Constants.Drive.ROTATION_DEADBAND)
            * Constants.Shooter.SHOOTER_OFFSET_RADIUS.in(Meters);
    // 90degrees CCW because CCW is positive and we want tangential velocity vector
    Translation2d transformationVector =
        new Translation2d(
            linearTangentSpeed * Constants.Shooter.HANG_TIME,
            DriveSubsystem.getInstance().getPose().getRotation().plus(Rotation2d.kCCW_90deg));
    return currentPos.plus(transformationVector);
  }

  public boolean isReadyToShoot() {
    return m_readytoShoot;
  }

  public boolean finishedZero() {
    return getInstance().m_finishedZero;
  }

  public void setFinishedZero(boolean value) {
    getInstance().m_finishedZero = value;
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
            getFuturePose(Constants.Shooter.HOOD_COLLISION_TIME),
            DriveSubsystem.getInstance().getPose().getRotation()));
    Logger.recordOutput("ShooterSubsystem/UnderTrench", robotCrossTrench());
    Logger.recordOutput("ShooterSubsystem/isReadyToShoot", isReadyToShoot());
  }
}
