// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class SerializationSubsystem extends StateMachine {

  public enum SerializationStates implements SystemState {
    CycleOff {
      @Override
      public void initialize() {
        getInstance().m_feedingRollerMotor.setControl(new VelocityVoltage(0));
        getInstance().m_shooterFeedLeaderMotor.setControl(new VelocityVoltage(0));
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_serializationState;
      }
    },
    CycleOn {
      @Override
      public void initialize() {
        getInstance()
            .m_feedingRollerMotor
            .setControl(
                new VelocityVoltage(Constants.SerializationConstants.FEEDING_ROLLER_MOTOR_SPEED));
        getInstance()
            .m_shooterFeedLeaderMotor
            .setControl(
                new VelocityVoltage(Constants.SerializationConstants.SHOOTER_FEED_MOTOR_SPEED));
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_serializationState;
      }
    },

    Reverse {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_serializationState;
      }
    }
  }

  private static SerializationSubsystem s_serializationInstance;
  private SerializationStates m_serializationState;

  private TalonFX m_feedingRollerMotor;
  private TalonFX m_shooterFeedLeaderMotor;
  private TalonFX m_shooterFeedFollowerMotor;

  public SerializationSubsystem() {
    super(SerializationStates.CycleOff);

    m_feedingRollerMotor = new TalonFX(Constants.MotorIdentification.FEEDING_ROLLER_MOTOR_ID);
    m_shooterFeedLeaderMotor =
        new TalonFX(Constants.MotorIdentification.SHOOTER_FEED_LEADER_MOTOR_ID);
    m_shooterFeedFollowerMotor =
        new TalonFX(Constants.MotorIdentification.SHOOTER_FEED_FOLLOWER_MOTOR_ID);

    TalonFXConfiguration feedingRollerConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterFeedLeaderConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterFeedFollowerConfig = new TalonFXConfiguration();

    m_feedingRollerMotor.getConfigurator().apply(feedingRollerConfig);
    m_shooterFeedLeaderMotor.getConfigurator().apply(shooterFeedLeaderConfig);
    m_shooterFeedFollowerMotor.getConfigurator().apply(shooterFeedFollowerConfig);

    m_shooterFeedFollowerMotor.setControl(
        new Follower(m_shooterFeedLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public static SerializationSubsystem getInstance() {
    if (s_serializationInstance == null) {
      s_serializationInstance = new SerializationSubsystem();
    }
    return s_serializationInstance;
  }

  public void setSerializationState(SerializationStates serializationState) {
    m_serializationState = serializationState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
