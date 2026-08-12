// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class TurretSubsystem extends StateMachine implements AutoCloseable {

  public enum TurretStates implements SystemState {
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

  private static TurretSubsystem s_turretInstance;

  public TurretSubsystem() {
    super(TurretStates.REST);
  }

  public static TurretSubsystem getInstance() {
    if (s_turretInstance == null) {
      s_turretInstance = new TurretSubsystem();
    }
    return s_turretInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() {}
}
