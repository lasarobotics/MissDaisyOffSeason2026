// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
      public void initialize() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ON {
      @Override
      public void execute() {
        // if (getInstance().getTarget() != null) {
        //   getInstance().setTurretPos(getInstance().getTurretPos(getInstance().getTarget())[0]);
        // }
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
    m_hoodConfig = new TalonFXConfiguration();
    m_hoodConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
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
    updateTurretEncoder();
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

  private Translation2d getTarget() {
    if (m_blueAlliance) {
      if (inAZ()) {
        return Constants.FieldConstants.BLUE_HUB_POS;
      } else if (inNZ()) {
        if (DriveSubsystem.getInstance().getPose().getY() < Constants.FieldConstants.NZ_MID_LINE) {
          return Constants.FieldConstants.BLUE_RIGHT_BUMP;
        }
        return Constants.FieldConstants.BLUE_LEFT_BUMP;
      }
    } else {
      if (inAZ()) {
        return Constants.FieldConstants.RED_HUB_POS;
      } else if (inNZ()) {
        if (DriveSubsystem.getInstance().getPose().getY() < Constants.FieldConstants.NZ_MID_LINE) {
          return Constants.FieldConstants.RED_LEFT_BUMP;
        }
        return Constants.FieldConstants.RED_RIGHT_BUMP;
      }
    }
    return null;
  }

  private double getTurretPos(Translation2d target) {
    Pose2d robotPose = DriveSubsystem.getInstance().getPose();
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

  // private double getHoodPos() {}

  // private void setHoodPos(double desiredPos) {}

  private void setTurretPos(double desiredPos) {
    m_turretMotor.setControl(
        m_positionVoltage.withPosition(
            desiredPos / (2 * Math.PI) * Constants.ShooterConstants.MOTOR_TURRET_GEAR_RATIO));
  }

  private boolean inNZ() {
    return DriveSubsystem.getInstance().getPose().getX() > Constants.FieldConstants.NZ_BLUE_X
        && DriveSubsystem.getInstance().getPose().getX() < Constants.FieldConstants.NZ_RED_X;
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

  @Override
  public void periodic() {
    m_blueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Logger.recordOutput("ShooterSubsystem/InNZ", inNZ());
    Logger.recordOutput("ShooterSubsystem/InAZ", inAZ());
    Logger.recordOutput("ShooterSubsystem/Target", getTarget());
    Logger.recordOutput(
        "ShooterSubsystem/TurretPos",
        new Pose2d(
            DriveSubsystem.getInstance().getTranslation2d(),
            new Rotation2d(getTurretPos(getTarget()))));
  }
  // This method will be called once per scheduler run
}
