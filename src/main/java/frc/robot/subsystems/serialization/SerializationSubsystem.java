// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.HeadHoncho;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

public class SerializationSubsystem extends StateMachine {

  public enum SerializationStates implements SystemState {
    REST {
      @Override
      public void initialize() {
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
        if (HeadHoncho.getInstance().ballChecksPass()) {
          getInstance().activateOmni(false);
          getInstance().activateMecanum(false);
        } else {
          getInstance().restOmni();
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
      public void initialize() {
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

  private VelocityVoltage m_velocityVoltage;

  public SerializationSubsystem() {
    super(SerializationStates.REST);
    setState(SerializationStates.REST);

    m_velocityVoltage = new VelocityVoltage(0);

    m_omniMotor = new TalonFX(Constants.Serialization.OMNI_CAN_ID);
    // TODO TODO TODO (TIKI TIKI TIKI)
    m_mecanumMotorLeader = new TalonFX(Constants.Serialization.MECANUM_FOLLOWER_CAN_ID);
    m_mecanumMotorFollower = new TalonFX(Constants.Serialization.MECANUM_LEADER_CAN_ID);

    TalonFXConfiguration omniConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKS(0.1953125).withKV(0.6607))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(6));

    m_omniMotor.getConfigurator().apply(omniConfig);

    final TalonFXConfiguration mecanumConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.1).withKS(0.2998046875).withKV(0.167))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.4285999536514282));

    m_mecanumMotorLeader.getConfigurator().apply(mecanumConfig);
    m_mecanumMotorFollower.getConfigurator().apply(mecanumConfig);

    m_mecanumMotorFollower.setControl(
        new Follower(m_mecanumMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void setState(SerializationStates state) {
    m_requestedState = state;
  }

  public static SerializationSubsystem getInstance() {
    if (s_serializationInstance == null) {
      s_serializationInstance = new SerializationSubsystem();
    }
    return s_serializationInstance;
  }

  public void restOmni() {
    getInstance().m_omniMotor.stopMotor();
  }

  public void restMecanum() {
    getInstance().m_mecanumMotorLeader.stopMotor();
  }

  public void activateOmni(boolean reverse) {
    AngularVelocity speed = Constants.Serialization.OMNI_SPEED.get().times(reverse ? -1 : 1);
    getInstance().m_omniMotor.setControl(getInstance().m_velocityVoltage.withVelocity(speed));
  }

  public void activateMecanum(boolean reverse) {
    AngularVelocity speed = Constants.Serialization.MECANUM_SPEED.get().times(reverse ? -1 : 1);
    // Voltage voltage = Constants.Serialization.MECANUM_SPEED.times(reverse ? -1 : 1);
    getInstance()
        .m_mecanumMotorLeader
        .setControl(getInstance().m_velocityVoltage.withVelocity(speed));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SerializationSubsystem/State", getState().toString());
    Logger.recordOutput(
        "SerializationSubsystem/omniCurrent", m_omniMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "SerializationSubsystem/mecanumCurrent",
        m_mecanumMotorLeader.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "SerializationSubsystem/mecanumSpeed",
        m_mecanumMotorLeader.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "SerializationSubsystem/omniSpeed", m_omniMotor.getVelocity().getValueAsDouble());
  }
}
