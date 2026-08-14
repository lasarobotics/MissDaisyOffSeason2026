// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class SerializationSubsystem extends StateMachine {

  public enum SerializationStates implements SystemState {
    OFF {
      @Override
      public void initialize() {
        getInstance()
            .m_serializationFeederLeader
            .setControl(getInstance().m_velocityVoltage.withVelocity(0));
        getInstance()
            .m_serializationOmni
            .setControl(getInstance().m_velocityVoltage.withVelocity(0));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ON {
      @Override
      public void initialize() {
        getInstance()
            .m_serializationFeederLeader
            .setControl(
                getInstance()
                    .m_velocityVoltage
                    .withVelocity(Constants.SerializationConstants.SERIALIZATION_FEEDER_SPEED));
        getInstance()
            .m_serializationOmni
            .setControl(
                getInstance()
                    .m_velocityVoltage
                    .withVelocity(Constants.SerializationConstants.SERIALIZATION_OMNI_SPEED));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    REVERSE {
      @Override
      public void initialize() {
        getInstance()
            .m_serializationFeederLeader
            .setControl(
                getInstance()
                    .m_velocityVoltage
                    .withVelocity(-Constants.SerializationConstants.SERIALIZATION_FEEDER_SPEED));
        getInstance()
            .m_serializationOmni
            .setControl(
                getInstance()
                    .m_velocityVoltage
                    .withVelocity(-Constants.SerializationConstants.SERIALIZATION_OMNI_SPEED));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static SerializationSubsystem s_serializationInstance;
  private SerializationStates m_selectedState;
  private TalonFX m_serializationFeederLeader;
  private TalonFX m_serializationFeederFollower;
  private TalonFX m_serializationOmni;
  private VelocityVoltage m_velocityVoltage;

  public SerializationSubsystem() {
    super(SerializationStates.OFF);
    m_serializationFeederLeader =
        new TalonFX(Constants.SerializationConstants.SERIALIZATION_FEEDER_LEADER_ID);
    m_serializationFeederFollower =
        new TalonFX(Constants.SerializationConstants.SERIALIZATION_FEEDER_LEADER_ID);
    m_serializationOmni = new TalonFX(Constants.SerializationConstants.SERIALIZATION_OMNI_ID);
    m_velocityVoltage = new VelocityVoltage(0);
    m_serializationFeederFollower.setControl(
        new Follower(m_serializationFeederLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public static SerializationSubsystem getInstance() {
    if (s_serializationInstance == null) {
      s_serializationInstance = new SerializationSubsystem();
    }
    return s_serializationInstance;
  }

  public void setState(SerializationStates state) {
    m_selectedState = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
