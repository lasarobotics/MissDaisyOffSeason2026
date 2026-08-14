// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class SerializationSubsystem extends StateMachine {

  public enum SerializationStates implements SystemState {
    REST {
      @Override
      public void execute() {
        getInstance().restSerialization();
      }

      @Override
      public SystemState nextState() {
        if (getInstance().m_isActive) {
          if (getInstance().m_isReversing) {
            return REVERSE;
          }
          return ACTIVE;
        }
        return this;
      }
    },

    ACTIVE {
      @Override
      public void execute() {
        getInstance().activateSerialization(false);
      }

      @Override
      public SystemState nextState() {
        if (!getInstance().m_isActive) {
          return REST;
        }
        if (getInstance().m_isReversing) {
          return REVERSE;
        }
        return this;
      }
    },

    REVERSE {
      @Override
      public void execute() {
        getInstance().activateSerialization(true);
      }

      @Override
      public SystemState nextState() {
        if (getInstance().m_isActive) {
          return ACTIVE;
        }
        if (!getInstance().m_isActive) {
          return REST;
        }
        return this;
      }
    },
  }

  private static SerializationSubsystem s_serializationInstance;

  private SerializationStates m_requestedState;

  private TalonFX m_omniMotor;
  private TalonFX m_mecanumMotorLeader;
  private TalonFX m_mecanumMotorFollower;

  private VelocityDutyCycle m_shooterVelocityDutyCycle;

  private boolean m_isActive;
  private boolean m_isReversing;

  public SerializationSubsystem() {
    super(SerializationStates.REST);

    m_isActive = true;
    m_isReversing = false;

    m_omniMotor = new TalonFX(Constants.SerializationConstants.OMNI_CAN_ID);
    m_mecanumMotorLeader = new TalonFX(Constants.SerializationConstants.MECANUM_LEADER_CAN_ID);
    m_mecanumMotorFollower = new TalonFX(Constants.SerializationConstants.MECANUM_FOLLOWER_CAN_ID);

    TalonFXConfiguration omniConfig = new TalonFXConfiguration();
    omniConfig.Slot0.withKP(0).withKI(0).withKD(0);
    m_omniMotor.getConfigurator().apply(omniConfig);

    TalonFXConfiguration mecanumConfig = new TalonFXConfiguration();
    mecanumConfig.Slot0.withKP(0).withKI(0).withKD(0);
    m_mecanumMotorLeader.getConfigurator().apply(mecanumConfig);
    m_mecanumMotorFollower.getConfigurator().apply(mecanumConfig);

    m_mecanumMotorFollower.setControl(
        new Follower(m_mecanumMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void setState(SerializationStates state) {
    getInstance().m_requestedState = state;
  }

  public static SerializationSubsystem getInstance() {
    if (s_serializationInstance == null) {
      s_serializationInstance = new SerializationSubsystem();
    }
    return s_serializationInstance;
  }

  public void setIsActive(boolean value) {
    getInstance().m_isActive = value;
    getInstance().m_isReversing = !value;
  }

  public void setIsReversing(boolean value) {
    getInstance().m_isReversing = value;
    getInstance().m_isActive = !value;
  }

  public void restSerialization() {
    getInstance()
        .m_omniMotor
        .setControl(
            getInstance()
                .m_shooterVelocityDutyCycle
                .withVelocity(Constants.SerializationConstants.OMNI_REST_SPEED));
    getInstance()
        .m_mecanumMotorLeader
        .setControl(
            getInstance()
                .m_shooterVelocityDutyCycle
                .withVelocity(Constants.SerializationConstants.MECANUM_REST_SPEED));
  }

  public void activateSerialization(boolean reverse) {
    double omniSpeed, mecanumSpeed;
    if (reverse) {
      omniSpeed = -Constants.SerializationConstants.OMNI_SPEED;
      mecanumSpeed = -Constants.SerializationConstants.MECANUM_SPEED;
    } else {
      omniSpeed = Constants.SerializationConstants.OMNI_SPEED;
      mecanumSpeed = Constants.SerializationConstants.MECANUM_SPEED;
    }
    getInstance()
        .m_omniMotor
        .setControl(getInstance().m_shooterVelocityDutyCycle.withVelocity(omniSpeed));
    getInstance()
        .m_mecanumMotorLeader
        .setControl(getInstance().m_shooterVelocityDutyCycle.withVelocity(mecanumSpeed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
