// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class IntakeSubsystem extends StateMachine {

  public enum IntakeStates implements SystemState {
    STOWED {
      @Override
      public void initialize() {
        getInstance().stow();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    },
    INACTIVE {
      @Override
      public void initialize() {
        getInstance().stopIntaking();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    },
    INTAKING {
      @Override
      public void initialize() {
        getInstance().deploy();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    }
  }

  public static void setState(IntakeStates nextState) {
    s_requestedNextState = nextState;
  }

  private static IntakeSubsystem s_intakeInstance;
  private static IntakeStates s_requestedNextState;

  private TalonFX m_intakeRollerLeader;
  private TalonFX m_intakeRollerFollower;
  private TalonFX m_armMotor;

  public IntakeSubsystem() {
    super(IntakeStates.INACTIVE);
    setState(IntakeStates.INACTIVE);

    m_intakeRollerLeader = new TalonFX(Constants.Intake.LEADER_ROLLER_ID);
    m_intakeRollerFollower = new TalonFX(Constants.Intake.FOLLOWER_ROLLER_ID);
    m_armMotor = new TalonFX(Constants.Intake.ARM_MOTOR_ID);

    m_intakeRollerFollower.setControl(
        new Follower(m_intakeRollerLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public static IntakeSubsystem getInstance() {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem();
    }
    return s_intakeInstance;
  }

  // TODO maybe do fancy running of roller to not be any faster than
  // drivetrain times 2?
  private void deploy() {
    m_intakeRollerLeader.set(Constants.Intake.ROLLER_SPEED);
    m_armMotor.setPosition(Constants.Intake.ARM_DEPLOY_POSITION);
  }

  private void stopIntaking() {
    m_intakeRollerLeader.set(0);
    m_armMotor.setPosition(Constants.Intake.ARM_DEPLOY_POSITION);
  }

  private void stow() {
    m_intakeRollerLeader.set(0);
    m_armMotor.setPosition(Constants.Intake.ARM_STOW_POSITION);
  }
}
