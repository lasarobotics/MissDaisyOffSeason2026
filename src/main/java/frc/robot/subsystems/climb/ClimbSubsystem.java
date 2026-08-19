// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class ClimbSubsystem extends StateMachine {

  public enum ClimbStates implements SystemState {
    REST {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    },

    READY {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    },

    CLIMB {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    },
  }

  private static ClimbSubsystem s_climbInstance;

  private ClimbStates m_requestedState;

  public ClimbSubsystem() {
    super(ClimbStates.REST);

    m_requestedState = ClimbStates.REST;
  }

  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null) {
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  public void setState(ClimbStates state) {
    getInstance().m_requestedState = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
