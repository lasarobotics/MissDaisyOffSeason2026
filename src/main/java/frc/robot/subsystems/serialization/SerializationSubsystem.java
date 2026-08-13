// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serialization;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class SerializationSubsystem extends StateMachine implements AutoCloseable {

  public enum SerializationStates implements SystemState {
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

  private static SerializationSubsystem s_serializationInstance;
  private SerializationStates m_selectedState;

  public SerializationSubsystem() {
    super(SerializationStates.OFF);
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

  @Override
  public void close() {}
}
