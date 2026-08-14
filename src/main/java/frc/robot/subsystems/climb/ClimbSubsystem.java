// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
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

  private TalonFX m_climbMotor;

  private PositionVoltage m_climbMotorRequest;

  public static void setState(ClimbStates nextState) {
    s_requestedNextState = nextState;
  }

  public ClimbSubsystem() {
    super(ClimbStates.RETRACTED);
    setState(ClimbStates.RETRACTED);

    m_climbMotor = new TalonFX(Constants.Climb.CLIMB_MOTOR_ID);

    m_climbMotorRequest = new PositionVoltage(0);
  }

  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null) {
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  private void retractClimber() {
    m_climbMotor.setControl(
        m_climbMotorRequest.withPosition(Constants.Climb.CLIMB_RETRACTED_SETPOINT));
  }

  private void extendClimber() {
    m_climbMotor.setControl(
        m_climbMotorRequest.withPosition(Constants.Climb.CLIMB_EXTENDED_SETPOINT));
  }
}
