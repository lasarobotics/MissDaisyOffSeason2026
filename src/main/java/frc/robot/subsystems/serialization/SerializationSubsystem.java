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
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SerializationSubsystem extends StateMachine {

  public enum SerializationStates implements SystemState {
    REST {
      @Override
      public void execute() {
        getInstance().restOmni();
        getInstance().restMecanum();
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    ACTIVE {
      @Override
      public void execute() {
        getInstance().activateOmni(false);
        if (ShooterSubsystem.getInstance().isShooterReady()) {
          getInstance().activateMecanum(false);
        } else {
          getInstance().restMecanum();
        }
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    REVERSE {
      @Override
      public void execute() {
        getInstance().activateOmni(true);
        getInstance().activateMecanum(true);
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },
  }

  private static SerializationSubsystem s_serializationInstance;

  private SerializationStates m_requestedState;

  private TalonFX m_omniMotor;
  private TalonFX m_mecanumMotorLeader;
  private TalonFX m_mecanumMotorFollower;

  private VelocityDutyCycle m_shooterVelocityDutyCycle;

  public SerializationSubsystem() {
    super(SerializationStates.REST);

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

  public void restOmni() {
    getInstance()
        .m_omniMotor
        .setControl(
            getInstance()
                .m_shooterVelocityDutyCycle
                .withVelocity(Constants.SerializationConstants.OMNI_REST_SPEED));
  }

  public void restMecanum() {
    getInstance()
        .m_mecanumMotorLeader
        .setControl(
            getInstance()
                .m_shooterVelocityDutyCycle
                .withVelocity(Constants.SerializationConstants.MECANUM_REST_SPEED));
  }

  public void activateOmni(boolean reverse) {
    double speed =
        (reverse)
            ? -Constants.SerializationConstants.OMNI_SPEED
            : Constants.SerializationConstants.OMNI_SPEED;
    getInstance()
        .m_omniMotor
        .setControl(getInstance().m_shooterVelocityDutyCycle.withVelocity(speed));
  }

  public void activateMecanum(boolean reverse) {
    double speed =
        (reverse)
            ? -Constants.SerializationConstants.MECANUM_SPEED
            : Constants.SerializationConstants.MECANUM_SPEED;
    getInstance()
        .m_mecanumMotorLeader
        .setControl(getInstance().m_shooterVelocityDutyCycle.withVelocity(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
