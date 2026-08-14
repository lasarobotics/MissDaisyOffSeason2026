// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class IntakeSubsystem extends StateMachine {

  public enum IntakeStates implements SystemState {
    REST {
      @Override
      public void execute() {
        getInstance().deployIntake();
        getInstance().stopIntake();
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    STOW {
      @Override
      public void execute() {
        getInstance().stowIntake();
        getInstance().stopIntake();
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    },

    INTAKE {
      @Override
      public void execute() {
        getInstance().deployIntake();
        getInstance().activateIntake(false);
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    },

    REVERSE {
      @Override
      public void execute() {
        getInstance().deployIntake();
        getInstance().activateIntake(true);
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    },
  }

  private static IntakeSubsystem s_intakeInstance;

  private IntakeStates m_requestedState;

  private TalonFX m_armMotor;
  private TalonFX m_intakeMotorLeader;
  private TalonFX m_intakeMotorFollower;

  private VelocityDutyCycle m_shooterVelocityDutyCycle;

  public IntakeSubsystem() {
    super(IntakeStates.INTAKE);

    m_armMotor = new TalonFX(Constants.IntakeConstants.ARM_CAN_ID);
    m_intakeMotorLeader = new TalonFX(Constants.IntakeConstants.LEADER_CAN_ID);
    m_intakeMotorFollower = new TalonFX(Constants.IntakeConstants.FOLLOWER_CAN_ID);

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.Slot0.withKP(0).withKI(0).withKD(0);
    m_armMotor.getConfigurator().apply(armConfig);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.Slot0.withKP(0).withKI(0).withKD(0);
    m_intakeMotorLeader.getConfigurator().apply(intakeConfig);
    m_intakeMotorFollower.getConfigurator().apply(intakeConfig);
    m_intakeMotorFollower.setControl(
        new Follower(m_intakeMotorLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public static IntakeSubsystem getInstance() {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem();
    }
    return s_intakeInstance;
  }

  public void setState(IntakeStates state) {
    getInstance().m_requestedState = state;
  }

  public void stopIntake() {
    getInstance()
        .m_intakeMotorLeader
        .setControl(
            getInstance()
                .m_shooterVelocityDutyCycle
                .withVelocity(Constants.IntakeConstants.INTAKE_STOW_SPEED));
  }

  public void activateIntake(boolean reverse) {
    double intakeSpeed =
        (reverse)
            ? -Constants.IntakeConstants.INTAKE_SPEED
            : Constants.IntakeConstants.INTAKE_SPEED;
    getInstance()
        .m_intakeMotorLeader
        .setControl(getInstance().m_shooterVelocityDutyCycle.withVelocity(intakeSpeed));
  }

  public void deployIntake() {
    getInstance().m_armMotor.setControl(Constants.IntakeConstants.ARM_DEPLOY_SETPOINT);
  }

  public void stowIntake() {
    getInstance().m_armMotor.setControl(Constants.IntakeConstants.ARM_STOW_SETPOINT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
