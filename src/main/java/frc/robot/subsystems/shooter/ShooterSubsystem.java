// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

public class ShooterSubsystem extends StateMachine {

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
    },

    SHOOT {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    },

    PASS_AZ {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return REST;
      }
    },

    PASS_NZ {
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

  private static ShooterSubsystem s_shooterInstance;

  private ShooterStates m_requestedState;

  public ShooterSubsystem() {
    super(ShooterStates.REST);
  }

  public double getDesiredShooterVelocity() {
    return 0;
  }

  public Angle getDesiredHoodAngle() {
    return Degrees.of(0);
  }

  public boolean atGoodShootingPosition() {
    return true;
  }

  public static ShooterSubsystem getInstance() {
    if (s_shooterInstance == null) {
      s_shooterInstance = new ShooterSubsystem();
    }
    return s_shooterInstance;
  }

  public void setState(ShooterStates state) {
    getInstance().m_requestedState = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
