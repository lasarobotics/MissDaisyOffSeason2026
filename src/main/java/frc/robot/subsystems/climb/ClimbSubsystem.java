// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class ClimbSubsystem extends StateMachine {

  public enum ClimbStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        getInstance()
            .m_climbMotor
            .setControl(new PositionVoltage(Constants.ClimbConstants.RETRACTED_POS));
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_climbState;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        getInstance()
            .m_climbMotor
            .setControl(new PositionVoltage(Constants.ClimbConstants.CLIMB_POS));
      }

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        if (getInstance().m_climbState == REST) {
          return REST;
        }
        return CLIMB;
      }
    }
  }

  private static ClimbSubsystem s_climbInstance;
  private ClimbStates m_climbState = ClimbStates.REST;

  private TalonFX m_climbMotor;

  public ClimbSubsystem() {
    super(ClimbStates.REST);

    m_climbMotor = new TalonFX(Constants.MotorIdentification.CLIMB_MOTOR_ID);

    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    m_climbMotor.getConfigurator().apply(climbConfig);
  }

  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null) {
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  public void setClimbState(ClimbStates climbState) {
    m_climbState = climbState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
