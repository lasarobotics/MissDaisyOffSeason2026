package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MotorIdentification;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShooterSubsystem extends StateMachine {
  public enum ShooterStates implements SystemState {

    CycleOff {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        ShooterSubsystem shooter = getInstance();

        shooter.m_shooterSpeedLeader.setControl(
            shooter.m_shooterVelocityRequest.withVelocity(0));
      }

      @Override
      public ShooterStates nextState() {
        return getInstance().m_shooterState;
      }
    },

    CycleOn {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        ShooterSubsystem shooter = getInstance();

        shooter.updateShootingTarget();
        shooter.updateShooterValues();

        shooter.setTurretPosition();
        shooter.setHoodAngle();

        shooter.m_shooterSpeedLeader.setControl(
            shooter.m_shooterVelocityRequest
                .withVelocity(shooter.m_rollerSpeeds));
      }

      @Override
      public ShooterStates nextState() {
        return getInstance().m_shooterState;
      }
    },

    Reverse {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        ShooterSubsystem shooter = getInstance();

        shooter.m_shooterSpeedLeader.setControl(
            shooter.m_shooterVelocityRequest.withVelocity(-5));
      }

      @Override
      public ShooterStates nextState() {
        return getInstance().m_shooterState;
      }
    }
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }

    return s_shooterInstance;
  }

  private static ShooterSubsystem s_shooterInstance;
  private ShooterStates m_shooterState = ShooterStates.CycleOff;

  private final TalonFX m_turretMotor;
  private final TalonFX m_shooterSpeedLeader;
  private final TalonFX m_shooterSpeedFollower;
  private final TalonFX m_hoodMotor;

  private final CANcoder encoder1;
  private final CANcoder encoder2;

  private final PositionVoltage m_turretPositionRequest =
      new PositionVoltage(0);
  private final PositionVoltage m_hoodPositionRequest =
      new PositionVoltage(0);
  private final VelocityVoltage m_shooterVelocityRequest =
      new VelocityVoltage(0);

  private Translation2d m_shootingTarget =
      new Translation2d();
  private double m_turretAngleRelativeRobot = 0;
  private double m_hoodAngle = 0;
  private double m_rollerSpeeds = 0;
  private double m_distanceToTarget = 0;
  private double m_shooterHorizontalVelocity = 0;
  private double m_verticalVelocity = 0;
  private double m_exitVelocity = 0;

  public ShooterSubsystem() {
    super(ShooterStates.CycleOff);

    m_turretMotor =
        new TalonFX(
            MotorIdentification.TURRET_MOTOR_ID);
    m_shooterSpeedLeader =
        new TalonFX(
            MotorIdentification.SHOOTER_SPEED_LEADER_MOTOR_ID);
    m_shooterSpeedFollower =
        new TalonFX(
            MotorIdentification.SHOOTER_SPEED_FOLLOWER_MOTOR_ID);
    m_hoodMotor =
        new TalonFX(
            MotorIdentification.HOOD_ANGLE_MOTOR_ID);
    encoder1 =
        new CANcoder(
            MotorIdentification.ENCODER1);
    encoder2 =
        new CANcoder(
            MotorIdentification.ENCODER2);

    TalonFXConfiguration turretConfig =
        new TalonFXConfiguration();
    TalonFXConfiguration shooterConfig =
        new TalonFXConfiguration();
    TalonFXConfiguration hoodConfig =
        new TalonFXConfiguration();

    m_turretMotor.getConfigurator().apply(turretConfig);
    m_shooterSpeedLeader.getConfigurator().apply(shooterConfig);
    m_hoodMotor.getConfigurator().apply(hoodConfig);

    m_shooterSpeedFollower.setControl(
        new Follower(
            MotorIdentification.SHOOTER_SPEED_LEADER_MOTOR_ID,
            MotorAlignmentValue.Opposed));
  }

  public void setShooterState(ShooterStates shooterState) {
    m_shooterState = shooterState;
  }

  public void updateShootingTarget() 
  {
    Pose2d robotPose =
        DriveSubsystem.getInstance().getPose();

    if (robotPose == null) {
      return;
    }

    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(null);

    if (alliance == DriverStation.Alliance.Blue) {
      if (robotPose.getX()
          < HubConstants.BLUE_HUB_POS.getX()) {
        m_shootingTarget =
            HubConstants.BLUE_HUB_POS;
      } else {
        double targetY;
        if (robotPose.getY()
            < HubConstants.BLUE_HUB_POS.getY()) {
          targetY = HubConstants.LEFTY_POS;
        } else {
          targetY = HubConstants.RIGHTY_POS;
        }
        m_shootingTarget =
            new Translation2d(
                HubConstants.BLUE_HUB_POS.getX(),
                targetY);
      }
    } else if (alliance == DriverStation.Alliance.Red) {
      if (robotPose.getX()
          > HubConstants.RED_HUB_POS.getX()) {
        m_shootingTarget =
            HubConstants.RED_HUB_POS;
      } else {
        double targetY;
        if (robotPose.getY()
            < HubConstants.RED_HUB_POS.getY()) {
          targetY = HubConstants.LEFTY_POS;
        } else {
          targetY = HubConstants.RIGHTY_POS;
        }

        m_shootingTarget =
            new Translation2d(
                HubConstants.RED_HUB_POS.getX(),
                targetY);
      }
    }
  }

  public void updateShooterValues() {

    Pose2d robotPose =
        DriveSubsystem.getInstance().getPose();

    if (robotPose == null
        || m_shootingTarget == null) {
      return;
    }

    Translation2d robotToTarget =
        m_shootingTarget.minus(
            robotPose.getTranslation());

    m_distanceToTarget =
        robotToTarget.getNorm();

    double targetAngle =
        Math.atan2(
            robotToTarget.getY(),
            robotToTarget.getX());

 
    double targetHeight = 6.0;
    double maxBallYPos = 8.0;
    double stationaryXVelocity =
        getVelocityXStationary(
            m_distanceToTarget,
            targetHeight,
            maxBallYPos);
    double stationaryYVelocity =
        getVelocityYStationary(
            maxBallYPos);

    double desiredBallVelocityX =
        stationaryXVelocity
            * Math.cos(targetAngle);
    double desiredBallVelocityY =
        stationaryXVelocity
            * Math.sin(targetAngle);

    ChassisSpeeds robotVelocity =
        DriveSubsystem.getInstance()
            .getFieldRelativeSpeeds();
    double shooterVelocityX =
        desiredBallVelocityX
            - robotVelocity.vxMetersPerSecond;
    double shooterVelocityY =
        desiredBallVelocityY
            - robotVelocity.vyMetersPerSecond;

    m_shooterHorizontalVelocity =
        Math.hypot(
            shooterVelocityX,
            shooterVelocityY);

    double shooterFieldAngle =
        Math.atan2(
            shooterVelocityY,
            shooterVelocityX);

    m_turretAngleRelativeRobot =
        wrapRadians(
            shooterFieldAngle
                - robotPose.getRotation().getRadians());

    m_verticalVelocity =
        stationaryYVelocity;

    m_exitVelocity =
        Math.hypot(
            m_shooterHorizontalVelocity,
            m_verticalVelocity);

    m_hoodAngle =
        Math.atan2(
            m_verticalVelocity,
            m_shooterHorizontalVelocity);

    double angularVelocity =
        (2.0 * m_exitVelocity)
            / (0.0508 + 0.0254);
    m_rollerSpeeds =
        angularVelocity
            / (2.0 * Math.PI);
  }

  public double getVelocityXStationary(
      double distance,
      double targetHeight,
      double maxBallYPos) {

    return distance * Math.sqrt(9.81)
        / (Math.sqrt(2 * maxBallYPos)
            + Math.sqrt(
                2 * (maxBallYPos - targetHeight)));
  }

  public double getVelocityYStationary(
      double maxBallYPos) {

    return Math.sqrt(
        maxBallYPos * 2 * 9.81);
  }

  public void setTurretPosition() {

    double motorPosition =
        m_turretMotor
            .getPosition()
            .getValueAsDouble();

    double currentTurretRotations =
        motorPosition
            / ShooterConstants.MOTOR_TURRET_GEAR_RATIO;

    double currentTurretAngle =
        currentTurretRotations
            * 2.0
            * Math.PI;

    double lowerBound =
        Math.toRadians(
            ShooterConstants.ANGLE_LOWER_BOUND);

    double upperBound =
        Math.toRadians(
            ShooterConstants.ANGLE_HIGHER_BOUND);

    double bestAngle = Double.NaN;

    double smallestDifference =
        Double.POSITIVE_INFINITY;

    for (int k = -3; k <= 3; k++) {

      double candidateAngle =
          m_turretAngleRelativeRobot
              + k * 2.0 * Math.PI;

      if (candidateAngle < lowerBound
          || candidateAngle > upperBound) {

        continue;
      }

      double difference =
          Math.abs(
              candidateAngle
                  - currentTurretAngle);

      if (difference < smallestDifference) {

        smallestDifference = difference;
        bestAngle = candidateAngle;
      }
    }

    if (Double.isNaN(bestAngle)) {
      return;
    }

    double desiredTurretRotations =
        bestAngle
            / (2.0 * Math.PI);

    double desiredMotorPosition =
        desiredTurretRotations
            * ShooterConstants.MOTOR_TURRET_GEAR_RATIO;

    m_turretMotor.setControl(
        m_turretPositionRequest
            .withPosition(desiredMotorPosition));
  }

  public void setHoodAngle() {
    double desiredHoodRotations =
        m_hoodAngle
            / (2.0 * Math.PI);

    double desiredMotorPosition =
        desiredHoodRotations
            * ShooterConstants.MOTOR_HOOD_GEAR_RATIO;

    m_hoodMotor.setControl(
        m_hoodPositionRequest
            .withPosition(desiredMotorPosition));
  }

  public void checkTurretPosition() {
    double[] possibilities1 =
        new double[ShooterConstants.TURRET_TEETH];
    double[] possibilities2 =
        new double[ShooterConstants.TURRET_TEETH];

    double encoder1Position =
        encoder1
            .getAbsolutePosition()
            .getValueAsDouble();
    double encoder2Position =
        encoder2
            .getAbsolutePosition()
            .getValueAsDouble();

    for (int i = 0;
        i < ShooterConstants.TURRET_TEETH;
        i++) {
      possibilities1[i] =
          (i + encoder1Position)
              * ((double)
                  ShooterConstants.GEAR1_TEETH
                  / ShooterConstants.TURRET_TEETH);
    }

    for (int i = 0;
        i < ShooterConstants.TURRET_TEETH;
        i++) {
      possibilities2[i] =
          (i + encoder2Position)
              * ((double)
                  ShooterConstants.GEAR2_TEETH
                  / ShooterConstants.TURRET_TEETH);
    }

    double bestPosition = -1;
    double bestError =
        Double.POSITIVE_INFINITY;

    for (int i = 0;
        i < ShooterConstants.TURRET_TEETH;
        i++) {
      for (int j = 0;
          j < ShooterConstants.TURRET_TEETH;
          j++) {
        double error =
            Math.abs(
                possibilities1[i]
                    - possibilities2[j]);
        if (error < bestError) {
          bestError = error;
          bestPosition =
              (possibilities1[i]
                  + possibilities2[j])
                  / 2.0;
        }
      }
    }

    if (bestError <= 0.01) {

      double motorPosition =
          bestPosition
              * ShooterConstants.MOTOR_TURRET_GEAR_RATIO;

      m_turretMotor.setPosition(
          motorPosition);
    }
  }

  private static double wrapRadians(
      double angle) {

    while (angle > Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2.0 * Math.PI;
    }

    return angle;
  }

public double getTurretAngleRelativeRobot() {
  return m_turretAngleRelativeRobot;
}

  @Override
  public void periodic() {
  }
}