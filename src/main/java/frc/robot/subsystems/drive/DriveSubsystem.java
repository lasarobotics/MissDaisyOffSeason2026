// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

  public enum DriveStates implements SystemState {
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

  private static DriveSubsystem s_driveInstance;

  public DriveSubsystem() {
    super(DriveStates.REST);
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() {}
}
