// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants;
import frc.robot.HeadHoncho;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShooterSubsystem extends StateMachine {

  public enum ShooterStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        getInstance().stop();
      }

      @Override
      public SystemState nextState() {
        if (DriveSubsystem.underTrench()) {
          return HOOD_DOWN;
        }

        return s_requestedNextState;
      }
    },
    SHOOTING {
      @Override
      public void execute() {
        getInstance().runShooter();
      }

      @Override
      public SystemState nextState() {
        if (DriveSubsystem.underTrench()) {
          return HOOD_DOWN;
        }

        return s_requestedNextState;
      }
    },
    HOOD_DOWN {
      @Override
      public void execute() {
        getInstance().lowerHood();
      }

      @Override
      public SystemState nextState() {
        if (!DriveSubsystem.underTrench()) {
          return s_requestedNextState;
        }

        return this;
      }
    }
  }

  public static void setState(ShooterStates nextState) {
    s_requestedNextState = nextState;
  }

  private static ShooterSubsystem s_shooterInstance;
  private static ShooterStates s_requestedNextState;

  private TalonFX m_shooterMotorLeader;
  private TalonFX m_shooterMotorFollower;
  private TalonFX m_hoodMotor;
  private TalonFX m_turretMotor;

  private VelocityVoltage m_shooterMotorRequest;
  private PositionVoltage m_hoodMotorRequest;
  private PositionVoltage m_turretMotorRequest;

  // debounce falling edges
  private Debouncer shooterSpeedDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  public ShooterSubsystem() {
    super(ShooterStates.REST);
    setState(ShooterStates.REST);

    m_shooterMotorLeader = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_LEADER_ID);
    m_shooterMotorFollower = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_FOLLOWER_ID);
    m_hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
    m_turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID);

    m_shooterMotorRequest = new VelocityVoltage(0);
    m_hoodMotorRequest = new PositionVoltage(0);
    m_turretMotorRequest = new PositionVoltage(0);

    m_shooterMotorFollower
        .setControl(new Follower(m_shooterMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }
    return s_shooterInstance;
  }

  public boolean shooterReady() {
    return (shooterSpeedDebouncer
        .calculate(m_shooterMotorLeader.getClosedLoopError().isNear(0,
            Constants.Shooter.SHOOTER_ALLOWED_ERROR))
        && m_hoodMotor.getClosedLoopError().isNear(0, Constants.Shooter.HOOD_ALLOWED_ERROR)
        && m_turretMotor.getClosedLoopError().isNear(0, Constants.Shooter.TURRET_ALLOWED_ERROR));
  }

  private void runShooter() {
    m_shooterMotorLeader
        .setControl(m_shooterMotorRequest.withVelocity(HeadHoncho.getDesiredShooterSpeed()));
    m_hoodMotor.setControl(m_hoodMotorRequest.withPosition(HeadHoncho.getDesiredHoodAngle()));
    m_turretMotor.setControl(m_turretMotorRequest.withPosition(HeadHoncho.getDesiredTurretAngle()));
  }

  private void lowerHood() {
    m_shooterMotorLeader.set(0);
    m_hoodMotor
        .setControl(m_hoodMotorRequest.withPosition(Constants.Shooter.LOWERED_HOOD_POSITION));
    m_turretMotor.setControl(m_turretMotorRequest.withPosition(HeadHoncho.getDesiredTurretAngle()));
  }

  private void stop() {
    m_shooterMotorLeader.set(0);
    m_hoodMotor.set(0);
    m_turretMotor.set(0);
  }
}
