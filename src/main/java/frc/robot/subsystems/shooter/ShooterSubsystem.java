// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.MotorIdentification;
import frc.robot.Constants.ShooterConstants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShooterSubsystem extends StateMachine {

  public enum ShooterStates implements SystemState {
    CycleOff {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_shooterState;
      }
    },
    CycleOn {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        getInstance().updateShootingTarget();
        getInstance().setTurretPosition();
        getInstance().setHoodAngle();
        getInstance()
            .m_shooterSpeedLeaderMotor
            .setControl(new VelocityVoltage(getInstance().m_rollerSpeeds));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_shooterState;
      }
    },

    Reverse {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        getInstance().m_shooterSpeedLeaderMotor.setControl(new VelocityVoltage(-5));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_shooterState;
      }
    }
  }

  private static ShooterSubsystem s_shooterInstance;
  private ShooterStates m_shooterState;

  private TalonFX m_turretMotor;
  private TalonFX m_shooterSpeedLeaderMotor;
  private TalonFX m_shooterSpeedFollowerMotor;
  private TalonFX m_hoodAngleMotor;

  private CANcoder encoder1;
  private CANcoder encoder2;

  private double m_hoodAngle;
  private double m_rollerSpeeds;
  private double m_turretAngleRelativeRobot;

  private Translation2d m_shootingTarget;

  public ShooterSubsystem() {
    super(ShooterStates.CycleOff);

    m_turretMotor = new TalonFX(Constants.MotorIdentification.TURRET_MOTOR_ID);
    m_shooterSpeedLeaderMotor =
        new TalonFX(Constants.MotorIdentification.SHOOTER_SPEED_LEADER_MOTOR_ID);
    m_shooterSpeedFollowerMotor =
        new TalonFX(Constants.MotorIdentification.SHOOTER_SPEED_FOLLOWER_MOTOR_ID);
    m_hoodAngleMotor = new TalonFX(Constants.MotorIdentification.HOOD_ANGLE_MOTOR_ID);

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterSpeedLeaderConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterSpeedFollowerConfig = new TalonFXConfiguration();
    TalonFXConfiguration hoodAngleConfig = new TalonFXConfiguration();

    m_turretMotor.getConfigurator().apply(turretConfig);
    m_shooterSpeedLeaderMotor.getConfigurator().apply(shooterSpeedLeaderConfig);
    m_shooterSpeedFollowerMotor.getConfigurator().apply(shooterSpeedFollowerConfig);
    m_hoodAngleMotor.getConfigurator().apply(hoodAngleConfig);

    m_shooterSpeedFollowerMotor.setControl(
        new Follower(m_shooterSpeedLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    encoder1 = new CANcoder(MotorIdentification.ENCODER1);
    encoder2 = new CANcoder(MotorIdentification.ENCODER2);
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }
    return s_shooterInstance;
  }

  public void setShooterState(ShooterStates shooterState) {
    m_shooterState = shooterState;
  }

  public void checkTurretPosition() {

    double[] possibilities1 = new double[ShooterConstants.TURRET_TEETH];
    double[] possibilities2 = new double[ShooterConstants.TURRET_TEETH];

    for (int i = 0; i < ShooterConstants.TURRET_TEETH; i++) {
      double newValue =
          (i + encoder1.getAbsolutePosition().getValueAsDouble())
              * ((double) ShooterConstants.GEAR1_TEETH / ShooterConstants.TURRET_TEETH);
      possibilities1[i] = newValue;
    }

    for (int i = 0; i < ShooterConstants.TURRET_TEETH; i++) {
      double newValue =
          (i + encoder2.getAbsolutePosition().getValueAsDouble())
              * ((double) ShooterConstants.GEAR2_TEETH / ShooterConstants.TURRET_TEETH);
      possibilities2[i] = newValue;
    }

    double match = -1;

    outerloop:
    for (double i : possibilities1) {
      for (double j : possibilities2) {
        if (Math.abs(i - j) <= 0.01) {
          match = (i + j) / 2;
          break outerloop;
        }
      }
    }

    if (match != -1) m_turretMotor.setPosition(match);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // PH 2026 Aim

  private static double getVelocityXStationary(
      double distance, double targetHeight, double maxBallYPos) {
    double y_max = maxBallYPos;
    double y_end = targetHeight;
    double g = 9.81;

    double x_vel =
        distance * (Math.sqrt(g)) / (Math.sqrt(2 * y_max) + Math.sqrt(2 * (y_max - y_end)));
    return x_vel;
  }

  /**
   * Get the Y velocity of stationary ball shot
   *
   * @param maxBallYPos The Y value for the highest point of the ball's curve
   * @return Value of Y velocity of the ball when shot stationary
   */
  private static double getVelocityYStationary(double maxBallYPos) {
    double y_max = maxBallYPos;
    double g = 9.81;
    double y_vel = Math.sqrt(y_max * 2 * g);
    return y_vel;
  }

  private void updateShootingTarget() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      if (DriveSubsystem.getInstance().getPose().getX()
          < Constants.HubConstants.BLUE_HUB_POS.getX()) {
        m_shootingTarget = Constants.HubConstants.BLUE_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY()
            < Constants.HubConstants.BLUE_HUB_POS.getY()) {
          m_shootingTarget =
              new Translation2d(
                  Constants.HubConstants.BLUE_HUB_POS.getX(), Constants.HubConstants.LEFTY_POS);
        } else {
          m_shootingTarget =
              new Translation2d(
                  Constants.HubConstants.BLUE_HUB_POS.getX(), Constants.HubConstants.RIGHTY_POS);
        }
      }
    } else {
      if (DriveSubsystem.getInstance().getPose().getX()
          > Constants.HubConstants.RED_HUB_POS.getX()) {
        m_shootingTarget = Constants.HubConstants.RED_HUB_POS;
      } else {
        if (DriveSubsystem.getInstance().getPose().getY()
            < Constants.HubConstants.RED_HUB_POS.getY()) {
          m_shootingTarget =
              new Translation2d(
                  Constants.HubConstants.RED_HUB_POS.getX(), Constants.HubConstants.LEFTY_POS);
        } else {
          m_shootingTarget =
              new Translation2d(
                  Constants.HubConstants.RED_HUB_POS.getX(), Constants.HubConstants.RIGHTY_POS);
        }
      }
    }
  }

  private void updateShooterValues() {

    Pose2d robotPose = DriveSubsystem.getInstance().getPose();
    Translation2d robotToHub = m_shootingTarget.minus(robotPose.getTranslation());
    Rotation2d turretAngle = robotToHub.getAngle().minus(robotPose.getRotation());

    m_turretAngleRelativeRobot = turretAngle.getRadians();

    double distance = robotToHub.getNorm();
    double targetHeight = 6.0;
    double maxBallYPos = 8.0;

    double xVelocity = getVelocityXStationary(distance, targetHeight, maxBallYPos);
    double yVelocity = getVelocityYStationary(maxBallYPos);

    m_hoodAngle = Math.atan2(yVelocity, xVelocity);

    double exitVelocity = Math.hypot(xVelocity, yVelocity);

    double bigWheelRadius = 0.0508;
    double smallWheelRadius = 0.0254;
    double angularVelocity = (2 * exitVelocity) / (bigWheelRadius + smallWheelRadius);

    m_rollerSpeeds = angularVelocity / (2 * Math.PI);
  }

  private void setTurretPosition() {

    updateShooterValues();

    double currentMotorPosition = m_turretMotor.getPosition().getValueAsDouble();

    double currentTurretAngle =
        (currentMotorPosition / ShooterConstants.MOTOR_TURRET_GEAR_RATIO) * 2.0 * Math.PI;
    double desiredTurretAngle = m_turretAngleRelativeRobot;
    double targetTurretAngle = desiredTurretAngle;

    while (targetTurretAngle - currentTurretAngle > Math.PI) {
      targetTurretAngle -= 2.0 * Math.PI;
    }

    while (targetTurretAngle - currentTurretAngle < -Math.PI) {
      targetTurretAngle += 2.0 * Math.PI;
    }

    if (targetTurretAngle < ShooterConstants.ANGLE_LOWER_BOUND) {

      double flippedTarget = targetTurretAngle + 2.0 * Math.PI;

      if (flippedTarget <= ShooterConstants.ANGLE_HIGHER_BOUND) {
        targetTurretAngle = flippedTarget;
      } else {
        return;
      }
    }

    if (targetTurretAngle > ShooterConstants.ANGLE_HIGHER_BOUND) {

      double flippedTarget = targetTurretAngle - 2.0 * Math.PI;

      if (flippedTarget >= ShooterConstants.ANGLE_LOWER_BOUND) {
        targetTurretAngle = flippedTarget;
      } else {
        return;
      }
    }

    if (targetTurretAngle < ShooterConstants.ANGLE_LOWER_BOUND
        || targetTurretAngle > ShooterConstants.ANGLE_HIGHER_BOUND) {
      return;
    }

    double targetTurretRotations = targetTurretAngle / (2.0 * Math.PI);
    double targetMotorPosition = targetTurretRotations * ShooterConstants.MOTOR_TURRET_GEAR_RATIO;
    m_turretMotor.setControl(new PositionVoltage(targetMotorPosition));
  }

  private void setHoodAngle() {

    double hoodRotations = m_hoodAngle / (2.0 * Math.PI);
    double targetMotorPosition = hoodRotations * ShooterConstants.MOTOR_HOOD_GEAR_RATIO;
    m_hoodAngleMotor.setControl(new PositionVoltage(targetMotorPosition));
  }
}
