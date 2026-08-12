// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class ShooterSubsystem extends StateMachine implements AutoCloseable {

  public enum ShooterStates implements SystemState {
    REST {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    }
  }

  private static ShooterSubsystem s_shooterInstance;

  public ShooterSubsystem() {
    super(ShooterStates.REST);
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }
    return s_shooterInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() {}
}
