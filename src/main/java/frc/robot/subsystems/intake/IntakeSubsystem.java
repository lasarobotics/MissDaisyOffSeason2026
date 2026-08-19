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

public class IntakeSubsystem extends StateMachine {

  public enum IntakeStates implements SystemState {
    CycleOff {
      @Override
      public void initialize() {
        getInstance().m_intakeRollerLeaderMotor.setControl(new VelocityVoltage(0));
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_intakeState;
      }
    },

    CycleOn {
      @Override
      public void initialize() {
        getInstance()
            .m_intakeRollerLeaderMotor
            .setControl(new VelocityVoltage(Constants.IntakeConstants.INTAKE_ROLLER_MOTOR_SPEED));
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_intakeState;
      }
    },

    Reverse {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_intakeState;
      }
    }
  }

  private static IntakeSubsystem s_intakeInstance;
  private IntakeStates m_intakeState = IntakeStates.CycleOff;

  private TalonFX m_slapDownMotor;
  private TalonFX m_intakeRollerLeaderMotor;
  private TalonFX m_intakeRollerFollowerMotor;

  public IntakeSubsystem() {
    super(IntakeStates.CycleOff);

    m_slapDownMotor = new TalonFX(Constants.MotorIdentification.SLAP_DOWN_MOTOR_ID);
    m_intakeRollerLeaderMotor =
        new TalonFX(Constants.MotorIdentification.INTAKE_ROLLER_LEADER_MOTOR_ID);
    m_intakeRollerFollowerMotor =
        new TalonFX(Constants.MotorIdentification.INTAKE_ROLLER_FOLLOWER_MOTOR_ID);

    TalonFXConfiguration slapDownConfig = new TalonFXConfiguration();
    TalonFXConfiguration intakeRollerLeaderConfig = new TalonFXConfiguration();
    TalonFXConfiguration intakeRollerFollowerConfig = new TalonFXConfiguration();

    m_slapDownMotor.getConfigurator().apply(slapDownConfig);
    m_intakeRollerLeaderMotor.getConfigurator().apply(intakeRollerLeaderConfig);
    m_intakeRollerFollowerMotor.getConfigurator().apply(intakeRollerFollowerConfig);

    m_intakeRollerFollowerMotor.setControl(
        new Follower(m_intakeRollerLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public static IntakeSubsystem getInstance() {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem();
    }
    return s_intakeInstance;
  }

  public void setIntakeState(IntakeStates intatekState) {
    m_intakeState = intatekState;
  }

  public void slapDownIntake() {
    m_slapDownMotor.setControl(new PositionVoltage(Constants.IntakeConstants.SLAPDOWN_DOWN_POS));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
