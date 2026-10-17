// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        getInstance().m_shooterLeader.stopMotor();
        getInstance().m_hoodMotor.setControl(getInstance().m_hoodRequest.withPosition(0));
        getInstance().m_turretMotor.setControl(getInstance().m_turretRequest.withPosition(0));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ZERO {
      Timer zeroTimer = new Timer();
      Thread turretUpdateThread;

      @Override
      public void initialize() {
        getInstance().m_readytoShoot = false;
        getInstance().m_shooterLeader.stopMotor();

        getInstance()
            .m_hoodMotor
            .setControl(
                new VoltageOut(Constants.Shooter.ZERO_VOLTAGE.in(Volts))
                    .withIgnoreSoftwareLimits(true));
        getInstance().setTurretPos(getInstance().getTurretPosition());
        turretUpdateThread =
            new Thread(
                () -> {
                  try {
                    Thread.sleep(500);
                    // keep trying until it succeeds for 10 tries
                    int i = 0;
                    boolean success = false;
                    while (i < 10) {
                      success = getInstance().updateTurretEncoder();
                      if (success) {
                        break;
                      }
                      Thread.sleep(20);
                      i++;
                    }
                    if (!success) {
                      DriverStation.reportError("ERROR: TURRET NOT ZEROED!", false);
                    }
                  } catch (InterruptedException e) {
                  }
                });
        turretUpdateThread.start();

        zeroTimer.reset();
        zeroTimer.start();
      }

      @Override
      public void execute() {
        Logger.recordOutput(
            "ShooterSubsystem/Current",
            getInstance().m_hoodMotor.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput(
            "ShooterSubsystem/ResetCurrent", Constants.Shooter.ZERO_THRESHOLD.in(Amps));
        Logger.recordOutput(
            "ShooterSubsystem/atZeroThreshold",
            getInstance().m_hoodMotor.getTorqueCurrent().getValueAsDouble()
                <= Constants.Shooter.ZERO_THRESHOLD.in(Amps));
        Logger.recordOutput(
            "ShooterSubsystem/hoodVoltage",
            getInstance().m_hoodMotor.getTorqueCurrent().getValueAsDouble());
        if (zeroTimer.hasElapsed(Constants.Shooter.ZEROING_DELAY)
            && getInstance().m_hoodMotor.getTorqueCurrent().getValueAsDouble()
                <= Constants.Shooter.ZERO_THRESHOLD.in(Amps)
            && !turretUpdateThread.isAlive()) {
          getInstance().m_hoodMotor.setPosition(Constants.Shooter.HOOD_ZERO_SETPOINT);
          getInstance().setState(ShooterStates.OFF);
        }
      }

      @Override
      public void end(boolean interrupted) {
        turretUpdateThread.interrupt();
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ON {
      @Override
      public void execute() {
        Translation3d target = getInstance().getTarget();
        double hangTime =
            (Math.sqrt(
                            (Constants.Shooter.MAX_BALL_Y_POS.getAsDouble()
                                    - Constants.Limelight.TURRET_PIVOT_HEIGHT.in(Meters))
                                * 2)
                        + Math.sqrt(
                            2 * (Constants.Shooter.MAX_BALL_Y_POS.getAsDouble() - target.getZ())))
                    / Math.sqrt(Constants.Field.GRAVITY_VALUE)
                + Constants.Shooter.SOTM_LATENCY.getAsDouble();

        Pose2d currentPose = DriveSubsystem.getInstance().getPose();
        Translation2d linearFuturePose =
            getInstance()
                .getFuturePose(
                    currentPose.transformBy(
                        new Transform2d(
                            Constants.Shooter.SHOOTER_OFFSET_X,
                            Constants.Shooter.SHOOTER_OFFSET_Y,
                            new Rotation2d(0))),
                    hangTime);
        Translation2d fullyTransformedPose =
            getInstance().transformByTangentialRotationSpeed(linearFuturePose, hangTime);

        // Translation2d robotPose = DriveSubsystem.getInstance().getPose().getTranslation();

        Translation2d targetDiff = target.toTranslation2d().minus(fullyTransformedPose);
        double distance = targetDiff.getNorm();
        double x_vel =
            getVelocityXStationary(
                distance, target.getZ(), Constants.Shooter.MAX_BALL_Y_POS.getAsDouble());
        double y_vel = getVelocityYStationary(Constants.Shooter.MAX_BALL_Y_POS.getAsDouble());

        if (!Double.isFinite(x_vel) || !Double.isFinite(y_vel)) {
          getInstance().m_readytoShoot = false;
          return;
        }

        Logger.recordOutput("ShooterSubsystem/translation2dTarget", target.toTranslation2d());

        getInstance()
            .setTurretPos(
                getInstance()
                    .updateDesiredTurretPos(
                        target.toTranslation2d(),
                        new Pose2d(
                            fullyTransformedPose,
                            DriveSubsystem.getInstance().getPose().getRotation())));
        getInstance().setHoodPos(getInstance().updateDesiredHoodPos(x_vel, y_vel));
        getInstance().setShooterSpeed(getInstance().getShooterSpeed(x_vel, y_vel));

        getInstance().m_readytoShoot =
            getInstance().turretAtPosition()
                && getInstance().hoodAtPosition()
                && getInstance().shooterAtSpeed();

        Logger.recordOutput(
            "ShooterSubsystem/HoodAngle", getInstance().getDesiredHoodPos().in(Degrees));
        Logger.recordOutput(
            "ShooterSubsystem/FlywheelSpeed", getInstance().getShooterSpeed(x_vel, y_vel));
        Logger.recordOutput("ShooterSubsystem/fullyTransformedPose", fullyTransformedPose);
        Logger.recordOutput(
            "ShooterSubsystem/turretFieldRotation",
            new Pose2d(
                DriveSubsystem.getInstance().getPose().getTranslation(),
                DriveSubsystem.getInstance()
                    .getPose()
                    .getRotation()
                    .plus(
                        Rotation2d.fromDegrees(
                            Constants.Shooter.SHOOTER_ROTATION_OFFSET
                                .plus(getInstance().getTurretPosition())
                                .in(Degrees)))));
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
  private PositionVoltage m_hoodRequest;
  private MotionMagicVoltage m_turretRequest;

  private TalonFX m_turretMotor;

  private boolean m_blueAlliance;
  private boolean m_readytoShoot;

  private Angle m_desiredTurretPosition;
  private Angle m_desiredHoodPosition;

  public ShooterSubsystem() {
    super(ShooterStates.OFF);
    setState(ShooterStates.OFF);
    m_shooterLeader = new TalonFX(Constants.Shooter.SHOOTER_LEADER_ID);
    m_shooterFollower = new TalonFX(Constants.Shooter.SHOOTER_FOLLOWER_ID);
    m_hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
    m_turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID);
    m_encoderOne = new CANcoder(Constants.Shooter.ENCODER_ONE_ID);
    m_encoderTwo = new CANcoder(Constants.Shooter.ENCODER_TWO_ID);

    m_shooterRequest = new VelocityVoltage(0);
    m_hoodRequest = new PositionVoltage(0);
    m_turretRequest = new MotionMagicVoltage(0);

    final TalonFXConfiguration shooterConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.42).withKS(0.19).withKV(0.11749999970197678));

    final TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(50))
                    .withSupplyCurrentLimit(Amps.of(30)))
            .withSlot0(new Slot0Configs().withKP(160).withKS(0.240234375))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(133.6888885498047))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Rotations.of(0.07500000298023224))
                    .withReverseSoftLimitEnable(true));

    TalonFXConfiguration turretConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(85).withKS(0.2197265625))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(46))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Rotations.of(0.5))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.of(-0.5)))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5)));

    CANcoderConfiguration encoderOneConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1.0))
                    .withMagnetOffset(Rotations.of(-0.417236328125))
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

    CANcoderConfiguration encoderTwoConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1.0))
                    .withMagnetOffset(Rotations.of(-0.396728515625))
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

    m_shooterLeader.getConfigurator().apply(shooterConfig);
    m_shooterFollower.getConfigurator().apply(shooterConfig);
    m_hoodMotor.getConfigurator().apply(hoodConfig);
    m_turretMotor.getConfigurator().apply(turretConfig);
    m_encoderOne.getConfigurator().apply(encoderOneConfig);
    m_encoderTwo.getConfigurator().apply(encoderTwoConfig);

    m_turretMotor.getClosedLoopError().setUpdateFrequency(50);
    m_hoodMotor.getClosedLoopError().setUpdateFrequency(50);
    m_shooterLeader.getClosedLoopError().setUpdateFrequency(50);

    m_encoderOne.getAbsolutePosition().setUpdateFrequency(100);
    m_encoderTwo.getAbsolutePosition().setUpdateFrequency(100);

    m_desiredHoodPosition = Rotations.of(0);
    m_desiredTurretPosition = Rotations.of(0);

    new Thread(
            () -> {
              try {
                Thread.sleep(500);
                // keep trying until it succeeds for 20 tries
                int i = 0;
                boolean success = false;
                while (i < 20) {
                  success = getInstance().updateTurretEncoder();
                  if (success) {
                    break;
                  }
                  Thread.sleep(20);
                  i++;
                }
                if (!success) {
                  DriverStation.reportError("ERROR: TURRET NOT ZEROED!", false);
                }
              } catch (InterruptedException e) {
              }
            })
        .start();

    m_hoodMotor.setPosition(0.0);
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

  public boolean turretAtPosition() {
    return getTurretPosition()
        .isNear(getDesiredTurretPos(), Constants.Shooter.TURRET_ALLOWED_ERROR);
  }

  public boolean hoodAtPosition() {
    return Rotations.of(Math.abs(m_hoodMotor.getClosedLoopError().getValue()))
        .lte(Constants.Shooter.HOOD_ALLOWED_ERROR);
  }

  public boolean shooterAtSpeed() {
    return Math.abs(m_shooterLeader.getClosedLoopError().getValue())
        <= Constants.Shooter.SHOOTER_ALLOWED_ERROR.in(RotationsPerSecond);
  }

  public AngularVelocity getTurretAngularVelocity() {
    return m_turretMotor.getVelocity().getValue();
  }

  private Translation3d getTarget() {
    if (m_blueAlliance) {
      if (inAZ()) {
        return Constants.Field.BLUE_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY() < Constants.Field.NZ_MID_LINE_Y) {
          return Constants.Field.BLUE_RIGHT_PASS;
        }
        return Constants.Field.BLUE_LEFT_PASS;
      }
    } else {
      if (inAZ()) {
        return Constants.Field.RED_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY() < Constants.Field.NZ_MID_LINE_Y) {
          return Constants.Field.RED_LEFT_PASS;
        }
        return Constants.Field.RED_RIGHT_PASS;
      }
    }
  }

  private Angle updateDesiredTurretPos(Translation2d target, Pose2d robotPose) {
    if (target == null) {
      return Degrees.of(0);
    }
    double xOffset = target.getX() - robotPose.getX();
    double yOffset = target.getY() - robotPose.getY();
    Angle angleToTarget =
        Radians.of(robotPose.getRotation().getRadians() - Math.atan2(yOffset, xOffset));
    Angle turretDesired =
        Radians.of(
            MathUtil.angleModulus(
                angleToTarget
                    .times(-1)
                    .plus(Constants.Shooter.SHOOTER_ROTATION_OFFSET)
                    .in(Radians)));
    m_desiredTurretPosition = turretDesired;
    return turretDesired;
  }

  private Angle getDesiredTurretPos() {
    return m_desiredTurretPosition;
  }

  private void setTurretPos(Angle desiredPos) {
    m_turretMotor.setControl(m_turretRequest.withPosition(desiredPos));
  }

  private Angle updateDesiredHoodPos(double x_vel, double y_vel) {
    if (robotCrossTrench()) {
      m_desiredHoodPosition = Degrees.of(0);
      return Degrees.of(0);
    }
    Angle hoodAngle =
        Radians.of(((Math.PI / 2) - Math.atan2(y_vel, x_vel)))
            .minus(Constants.Shooter.HOOD_ZERO_ANGLE)
            .plus(Degrees.of(Constants.Shooter.HOOD_FUDGE_FACTOR.getAsDouble()));
    m_desiredHoodPosition = hoodAngle;
    return hoodAngle;
  }

  private Angle getDesiredHoodPos() {
    return m_desiredHoodPosition;
  }

  private void setHoodPos(Angle hoodPos) {
    m_hoodMotor.setControl(m_hoodRequest.withPosition(hoodPos));
  }

  private double getShooterSpeed(double x_vel, double y_vel) {
    double shootSpeed = Math.hypot(x_vel, y_vel);
    // 4/3 is because there's two wheels of different sizes
    // This is hardcoded based on the physical robot, but that's not changing
    // so it's not really worth programming the math
    double desiredRPS =
        (shootSpeed * 4 / 3) / (Inches.of(4).in(Meters) * Math.PI)
            + Constants.Shooter.FLYWHEEL_FUDGE_FACTOR.getAsDouble();
    return desiredRPS;
  }

  private void setShooterSpeed(double speed) {
    m_shooterLeader.setControl(m_shooterRequest.withVelocity(speed));
  }

  private static double getVelocityXStationary(
      double distance, double targetHeight, double maxBallYPos) {

    double g = Constants.Field.GRAVITY_VALUE;

    if (maxBallYPos <= Math.max(Constants.Limelight.TURRET_PIVOT_HEIGHT.in(Meters), targetHeight)) {
      return Double.NaN;
    }

    return distance
        * Math.sqrt(g)
        / (Math.sqrt(2 * (maxBallYPos - Constants.Limelight.TURRET_PIVOT_HEIGHT.in(Meters)))
            + Math.sqrt(2 * (maxBallYPos - targetHeight)));
  }

  private static double getVelocityYStationary(double maxBallYPos) {
    double g = Constants.Field.GRAVITY_VALUE;

    if (maxBallYPos <= Constants.Limelight.TURRET_PIVOT_HEIGHT.in(Meters)) {
      return Double.NaN;
    }

    return Math.sqrt(2 * g * (maxBallYPos - Constants.Limelight.TURRET_PIVOT_HEIGHT.in(Meters)));
  }

  public boolean robotCrossTrench() {
    Translation2d a = DriveSubsystem.getInstance().getPose().getTranslation();
    Translation2d b =
        getFuturePose(
            DriveSubsystem.getInstance().getPose(), Constants.Shooter.HOOD_COLLISION_TIME);
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

  private boolean updateTurretEncoder() {
    StatusSignal<Angle> encoderOneSignal = m_encoderOne.getAbsolutePosition();
    StatusSignal<Angle> encoderTwoSignal = m_encoderTwo.getAbsolutePosition();
    BaseStatusSignal.refreshAll(encoderOneSignal, encoderTwoSignal);
    encoderOneSignal.waitForUpdate(0.04);
    encoderTwoSignal.waitForUpdate(0.04);

    double encoderOnePosition = encoderOneSignal.getValue().in(Rotations);
    double encoderTwoPosition = encoderTwoSignal.getValue().in(Rotations);

    int checkRange = 5;

    double[] encoderOnePossible = new double[checkRange * 2];
    double[] encoderTwoPossible = new double[checkRange * 2];

    /*
     * Basically, the turret rotates from -0.5 to 0.5 rotations.
     * Every turret rotation is ~5.2 encoder rotations
     * so we check 5 encoder rotations in each direction to be safe
     */
    for (int i = -checkRange; i < checkRange; i++) {
      encoderOnePossible[i + checkRange] =
          (i + encoderOnePosition)
              * ((double) Constants.Shooter.ENCODER_ONE_TEETH
                  / Constants.Shooter.TURRET_GEAR_TEETH);
    }
    for (int i = -checkRange; i < checkRange; i++) {
      encoderTwoPossible[i + checkRange] =
          (i + encoderTwoPosition)
              * ((double) Constants.Shooter.ENCODER_TWO_TEETH
                  / Constants.Shooter.TURRET_GEAR_TEETH);
    }

    double bestError = Double.MAX_VALUE;
    double bestSolution = 0;
    for (double eOnePossible : encoderOnePossible) {
      for (double eTwoPossible : encoderTwoPossible) {
        if (Math.abs(eTwoPossible - eOnePossible) < bestError) {
          bestError = Math.abs(eTwoPossible - eOnePossible);
          bestSolution = (eOnePossible + eTwoPossible) / 2;
        }
      }
    }
    if (bestError <= Constants.Shooter.CRT_EPSILON) {
      m_turretMotor.setPosition(bestSolution);
      return true;
    }
    return false;
  }

  public Angle getTurretPosition() {
    return m_turretMotor.getPosition().getValue();
  }

  private Translation2d getFuturePose(Pose2d currentPose, double time) {
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
  private Translation2d transformByTangentialRotationSpeed(
      Translation2d currentPos, double hangTime) {
    Angle turretAngleOffset =
        Radians.of(
            Math.atan2(
                Constants.Shooter.SHOOTER_OFFSET_Y.in(Meters),
                Constants.Shooter.SHOOTER_OFFSET_X.in(Meters)));
    double omega = DriveSubsystem.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
    double linearTangentSpeed =
        (omega < Constants.Drive.ROTATION_DEADBAND ? 0 : omega)
            * Constants.Shooter.SHOOTER_OFFSET_RADIUS.in(Meters);
    // 90 degrees CCW because CCW is positive and we want tangential velocity vector
    Translation2d transformationVector =
        new Translation2d(
            linearTangentSpeed * hangTime,
            DriveSubsystem.getInstance()
                .getPose()
                .getRotation()
                .plus(Rotation2d.kCCW_90deg)
                .plus(Rotation2d.fromDegrees(turretAngleOffset.in(Degrees))));
    return currentPos.plus(transformationVector);
  }

  public boolean isReadyToShoot() {
    return m_readytoShoot;
  }

  @Override
  public void periodic() {
    m_blueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Logger.recordOutput("ShooterSubsystem/State", getState().toString());
    Logger.recordOutput("ShooterSubsystem/InNZ", inNZ());
    Logger.recordOutput("ShooterSubsystem/InAZ", inAZ());
    Logger.recordOutput("ShooterSubsystem/Target", getTarget());
    Logger.recordOutput(
        "ShooterSubsystem/FuturePoseHoodCollisionTime",
        new Pose2d(
            getFuturePose(
                DriveSubsystem.getInstance().getPose(), Constants.Shooter.HOOD_COLLISION_TIME),
            DriveSubsystem.getInstance().getPose().getRotation()));
    Logger.recordOutput("ShooterSubsystem/UnderTrench", robotCrossTrench());
    Logger.recordOutput("ShooterSubsystem/isReadyToShoot", isReadyToShoot());
    Logger.recordOutput("ShooterSubsystem/hoodAtPosition", hoodAtPosition());
    Logger.recordOutput("ShooterSubsystem/turretAtPosition", turretAtPosition());
    Logger.recordOutput("ShooterSubsystem/shooterAtSpeed", shooterAtSpeed());
    Logger.recordOutput("ShooterSubsystem/hoodPosition", m_hoodMotor.getPosition().getValue());
    Logger.recordOutput(
        "ShooterSubsystem/hoodTarget", m_hoodMotor.getClosedLoopReference().getValue());
    Logger.recordOutput(
        "ShooterSubsystem/shooterPosition", m_shooterLeader.getVelocity().getValue());
    Logger.recordOutput(
        "ShooterSubsystem/shooterTarget", m_shooterLeader.getClosedLoopReference().getValue());
    Logger.recordOutput("ShooterSubsystem/turretPosition", m_turretMotor.getPosition().getValue());
    Logger.recordOutput(
        "ShooterSubsystem/turretTarget", m_turretMotor.getClosedLoopReference().getValue());
    Logger.recordOutput(
        "ShooterSubsystem/hoodCurrent", m_hoodMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "ShooterSubsystem/shooterCurrent", m_shooterLeader.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "ShooterSubsystem/turretCurrent", m_turretMotor.getStatorCurrent().getValueAsDouble());
  }
}
