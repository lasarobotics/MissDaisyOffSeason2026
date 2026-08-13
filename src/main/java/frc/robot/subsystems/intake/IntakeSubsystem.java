// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class IntakeSubsystem extends StateMachine implements AutoCloseable {

  public enum IntakeStates implements SystemState {
    OFF {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    ON {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static IntakeSubsystem s_intakeInstance;
  private IntakeStates m_selectedState;

  public IntakeSubsystem() {
    super(IntakeStates.OFF);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() {}
}
