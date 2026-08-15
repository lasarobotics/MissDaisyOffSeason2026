// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serialization;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SerializationSubsystem extends StateMachine {

  public enum SerializationStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        getInstance().stop();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    },
    SERIALIZING {
      @Override
      public void execute() {
        if (ShooterSubsystem.getInstance().shooterReady()) {
          getInstance().runRollers();
        } else {
          getInstance().stop();
        }
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    }
  }

  public static void setState(SerializationStates nextState) {
    s_requestedNextState = nextState;
  }

  private static SerializationSubsystem s_serializationInstance;
  private static SerializationStates s_requestedNextState;

  private TalonFX m_omniWheelMotor;
  private TalonFX m_mecanumRollerLeader;
  private TalonFX m_mecanumRollerFollower;

  private VelocityVoltage m_omniWheelRequest;
  private VelocityVoltage m_mecanumRollerRequest;

  public SerializationSubsystem() {
    super(SerializationStates.REST);
    setState(SerializationStates.REST);

    m_omniWheelMotor = new TalonFX(Constants.Serialization.OMNI_WHEEL_MOTOR_ID);
    m_mecanumRollerLeader = new TalonFX(Constants.Serialization.LEADER_MECANUM_ROLLER_ID);
    m_mecanumRollerFollower = new TalonFX(Constants.Serialization.FOLLOWER_MECANUM_ROLLER_ID);

    m_omniWheelRequest = new VelocityVoltage(0);
    m_mecanumRollerRequest = new VelocityVoltage(0);

    m_mecanumRollerFollower
        .setControl(new Follower(m_mecanumRollerLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public static SerializationSubsystem getInstance() {
    if (s_serializationInstance == null) {
      s_serializationInstance = new SerializationSubsystem();
    }
    return s_serializationInstance;
  }

  private void runRollers() {
    m_omniWheelMotor
        .setControl(m_omniWheelRequest.withVelocity(RotationsPerSecond.of(IntakeSubsystem
            .getBallEntrySpeed().div(Constants.Serialization.OMNI_WHEEL_RADIUS.in(Meters))
            .times(Constants.Serialization.OMNI_WHEEL_SPEED_SCALAR).in(MetersPerSecond))));
    m_mecanumRollerLeader
        .setControl(m_mecanumRollerRequest.withVelocity(RotationsPerSecond.of(IntakeSubsystem
            .getBallEntrySpeed().div(Constants.Serialization.MECANUM_ROLLER_RADIUS.in(Meters))
            .times(Constants.Serialization.MECANUM_ROLLER_SPEED_SCALAR).in(MetersPerSecond))));
  }

  private void stop() {
    m_omniWheelMotor.set(0);
    m_mecanumRollerLeader.set(0);
  }
}
