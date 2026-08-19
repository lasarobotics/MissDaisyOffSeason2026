// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

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
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_shooterState;
      }
    },

    Reverse {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
