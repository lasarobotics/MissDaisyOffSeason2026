// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class ClimbSubsystem extends StateMachine {

  public enum ClimbStates implements SystemState {
    RETRACTED {
      @Override
      public void initialize() {
        getInstance().retractClimber();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    },
    EXTENDED {
      @Override
      public void initialize() {
        getInstance().extendClimber();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    }
  }

  private static ClimbSubsystem s_climbInstance;
  private static ClimbStates s_requestedNextState;

  public static void setState(ClimbStates nextState) {
    s_requestedNextState = nextState;
  }

  public ClimbSubsystem() {
    super(ClimbStates.RETRACTED);
    setState(ClimbStates.RETRACTED);
  }

  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null) {
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  private void retractClimber() {}

  private void extendClimber() {}
}
