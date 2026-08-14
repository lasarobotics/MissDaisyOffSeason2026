// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class ClimbSubsystem extends StateMachine {

  public enum ClimbStates implements SystemState {
    OFF {
      @Override
      public void initialize() {
        getInstance()
            .m_climbMotor
            .setControl(
                getInstance()
                    .m_positionVoltage
                    .withPosition(Constants.ClimbConstants.CLIMB_DOWN_POS));
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
            .m_climbMotor
            .setControl(
                getInstance()
                    .m_positionVoltage
                    .withPosition(Constants.ClimbConstants.CLIMB_UP_POS));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static ClimbSubsystem s_climbInstance;
  private ClimbStates m_selectedState;
  private TalonFX m_climbMotor;
  private TalonFXConfiguration m_climbConfig;
  private PositionVoltage m_positionVoltage;

  public ClimbSubsystem() {
    super(ClimbStates.OFF);
    m_climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID);
    m_positionVoltage =
        new PositionVoltage(0).withFeedForward(Constants.ClimbConstants.CLIMB_MOTOR_FEEDFORWARD);
    m_climbConfig = new TalonFXConfiguration();
    m_climbConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_climbMotor.getConfigurator().apply(m_climbConfig);
    m_climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  }

  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null) {
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  public void setState(ClimbStates state) {
    m_selectedState = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
