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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

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
      public void execute() {
        if (ShooterSubsystem.getInstance().robotCrossTrench()
            || DriveSubsystem.getInstance().underTower()
            || !ShooterSubsystem.getInstance().canSeeTarget()) {
          getInstance()
              .m_serializationFeederLeader
              .setControl(getInstance().m_velocityVoltage.withVelocity(0));
          getInstance()
              .m_serializationOmni
              .setControl(getInstance().m_velocityVoltage.withVelocity(0));
        } else {
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
  private TalonFXConfiguration m_motorConfig;

  public SerializationSubsystem() {
    super(SerializationStates.OFF);
    setState(SerializationStates.OFF);
    m_serializationFeederLeader =
        new TalonFX(Constants.SerializationConstants.SERIALIZATION_FEEDER_LEADER_ID);
    m_serializationFeederFollower =
        new TalonFX(Constants.SerializationConstants.SERIALIZATION_FEEDER_LEADER_ID);
    m_serializationOmni = new TalonFX(Constants.SerializationConstants.SERIALIZATION_OMNI_ID);
    m_velocityVoltage = new VelocityVoltage(0);
    m_serializationFeederFollower.setControl(
        new Follower(m_serializationFeederLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    m_motorConfig = new TalonFXConfiguration();
    m_motorConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_serializationFeederLeader
        .getConfigurator()
        .apply(m_motorConfig); // TODO add individual configs
    m_serializationFeederFollower.getConfigurator().apply(m_motorConfig);
    m_serializationOmni.getConfigurator().apply(m_motorConfig);
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
    // Logger.recordOutput(getName() + "/currentState", getState().toString());
    // Logger.recordOutput(getName() + "/selectedState", m_selectedState);
    Logger.recordOutput(
        getName() + "SerializationSubsystem/FeederSpeed",
        m_serializationFeederLeader.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        getName() + "SerializationSubsystem/OmniSpeed",
        m_serializationOmni.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "SerializationSubsystem/UnderTower", DriveSubsystem.getInstance().underTower());
    Logger.recordOutput(
        "SerializationSubsystem/CanSeeTarget", ShooterSubsystem.getInstance().canSeeTarget());
  }
}
