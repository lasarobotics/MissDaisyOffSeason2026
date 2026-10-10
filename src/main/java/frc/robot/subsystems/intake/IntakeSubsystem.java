// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends StateMachine {

  public enum IntakeStates implements SystemState {
    REST {
      @Override
      public void execute() {
        getInstance().deployIntake();
        getInstance().stopIntake();
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    ZERO {
      Timer m_zeroTimer = new Timer();

      @Override
      public void initialize() {
        getInstance().m_armMotor.setVoltage(Constants.Intake.ZERO_VOLTAGE.in(Volts));
        m_zeroTimer.reset();
        m_zeroTimer.start();
      }

      @Override
      public void execute() {
        if (m_zeroTimer.hasElapsed(Constants.Intake.ZEROING_DELAY)
            && getInstance().m_armMotor.getTorqueCurrent().getValueAsDouble()
                >= Constants.Intake.ZERO_THRESHOLD.in(Amps)) {
          getInstance().m_armMotor.setPosition(0.0);
          getInstance().setFinishedZero(true);
          getInstance().setState(INTAKE);
        }
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    STOW {
      @Override
      public void execute() {
        getInstance().stowIntake();
        getInstance().stopIntake();
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    INTAKE {
      @Override
      public void execute() {
        getInstance().deployIntake();
        getInstance().activateIntake(false);
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },

    REVERSE {
      @Override
      public void execute() {
        getInstance().deployIntake();
        getInstance().activateIntake(true);
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_requestedState;
      }
    },
  }

  private static IntakeSubsystem s_intakeInstance;

  private IntakeStates m_requestedState;

  private TalonFX m_armMotor;
  private TalonFX m_intakeMotorLeader;
  private TalonFX m_intakeMotorFollower;

  private VelocityVoltage m_velocityVoltage;

  private boolean m_finishedZero;

  private PositionVoltage m_positionRequest;

  public IntakeSubsystem() {
    super(IntakeStates.INTAKE);

    m_finishedZero = false;
    m_positionRequest = new PositionVoltage(0);

    m_requestedState = IntakeStates.INTAKE;

    m_velocityVoltage = new VelocityVoltage(0);

    m_armMotor = new TalonFX(Constants.Intake.ARM_CAN_ID);
    m_intakeMotorLeader = new TalonFX(Constants.Intake.LEADER_CAN_ID);
    m_intakeMotorFollower = new TalonFX(Constants.Intake.FOLLOWER_CAN_ID);

    TalonFXConfiguration armConfig =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(50)
                    .withKD(1)
                    .withKS(0.23046875)
                    .withKG(-0.599609375)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withGravityArmPositionOffset(Rotations.of(-0.2490234375)))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(34.97140121459961))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Rotations.of(0.30000001192092896))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Rotations.of(-0.10000000149011612)))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(2))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5))
                    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(40)));

    m_armMotor.getConfigurator().apply(armConfig);

    TalonFXConfiguration intakeConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(0.2).withKS(0.259765625).withKV(0.24))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2.037));

    m_intakeMotorLeader.getConfigurator().apply(intakeConfig);
    m_intakeMotorFollower.getConfigurator().apply(intakeConfig);
    m_intakeMotorFollower.setControl(
        new Follower(m_intakeMotorLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public static IntakeSubsystem getInstance() {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem();
    }
    return s_intakeInstance;
  }

  public void setState(IntakeStates state) {
    getInstance().m_requestedState = state;
  }

  public void stopIntake() {
    getInstance().m_intakeMotorLeader.stopMotor();
  }

  public void activateIntake(boolean reverse) {
    double intakeSpeed =
        (reverse)
            ? -Constants.Intake.INTAKE_ACTIVE_SPEED.in(RotationsPerSecond)
            : Constants.Intake.INTAKE_ACTIVE_SPEED.in(RotationsPerSecond);
    getInstance()
        .m_intakeMotorLeader
        .setControl(getInstance().m_velocityVoltage.withVelocity(intakeSpeed));
  }

  public void deployIntake() {
    getInstance()
        .m_armMotor
        .setControl(
            getInstance().m_positionRequest.withPosition(Constants.Intake.ARM_DEPLOY_SETPOINT));
  }

  public void stowIntake() {
    getInstance()
        .m_armMotor
        .setControl(
            getInstance().m_positionRequest.withPosition(Constants.Intake.ARM_STOW_SETPOINT));
  }

  public boolean finishedZero() {
    return getInstance().m_finishedZero;
  }

  public void setFinishedZero(boolean value) {
    getInstance().m_finishedZero = value;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("IntakeSubsystem/State", getState().toString());
  }
}
