// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends StateMachine {

  public enum IntakeStates implements SystemState {
    OFF {
      @Override
      public void initialize() {
        getInstance()
            .m_intakeRollerLeader
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
            .m_intakeRollerLeader
            .setControl(
                getInstance()
                    .m_velocityVoltage
                    .withVelocity(Constants.IntakeConstants.INTAKE_ROLLER_SPEED));
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
            .m_intakeRollerLeader
            .setControl(
                getInstance()
                    .m_velocityVoltage
                    .withVelocity(-Constants.IntakeConstants.INTAKE_ROLLER_SPEED));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static IntakeSubsystem s_intakeInstance;
  private IntakeStates m_selectedState;
  private final TalonFX m_intakeRollerLeader;
  private final TalonFX m_intakeRollerFollower;
  private final TalonFX m_intakeSlapdown;
  private final PositionVoltage m_slapdownPositionVoltage;
  private VelocityVoltage m_velocityVoltage;
  private TalonFXConfiguration m_rollerConfig;
  private TalonFXConfiguration m_slapdownConfig;

  public IntakeSubsystem() {
    super(IntakeStates.OFF);
    setState(IntakeStates.OFF);
    m_intakeRollerFollower = new TalonFX(Constants.IntakeConstants.INTAKE_ROLLER_FOLLOWER_ID);
    m_intakeRollerLeader = new TalonFX(Constants.IntakeConstants.INTAKE_ROLLER_LEADER_ID);
    m_intakeSlapdown = new TalonFX(Constants.IntakeConstants.INTAKE_SLAPDOWN_ID);
    m_intakeRollerFollower.setControl(
        new Follower(m_intakeRollerLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    m_slapdownPositionVoltage = new PositionVoltage(0);
    m_velocityVoltage = new VelocityVoltage(0);
    m_rollerConfig = new TalonFXConfiguration();
    m_slapdownConfig = new TalonFXConfiguration();
    m_rollerConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_slapdownConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_intakeRollerLeader.getConfigurator().apply(m_rollerConfig); // TODO add individual configs
    m_intakeRollerFollower.getConfigurator().apply(m_rollerConfig);
    m_intakeSlapdown.getConfigurator().apply(m_slapdownConfig);
  }

  public static IntakeSubsystem getInstance() {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem();
    }
    return s_intakeInstance;
  }

  public void setState(IntakeStates state) {
    m_selectedState = state;
  }

  public void deploy() {
    m_intakeSlapdown.setControl(
        m_slapdownPositionVoltage.withPosition(Constants.IntakeConstants.SLAPDOWN_POS));
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/currentState", getState().toString());
    Logger.recordOutput(getName() + "/selectedState", m_selectedState);
    Logger.recordOutput(
        getName() + "/slapdownPos", m_intakeSlapdown.getPosition().getValueAsDouble());
    Logger.recordOutput(
        getName() + "/rollerSpeed", m_intakeRollerLeader.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
