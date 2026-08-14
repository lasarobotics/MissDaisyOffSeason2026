// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.AimUtil;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;

public class IntakeSubsystem extends StateMachine {

  public enum IntakeStates implements SystemState {
    STOWED {
      @Override
      public void initialize() {
        getInstance().stow();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    },
    INACTIVE {
      @Override
      public void initialize() {
        getInstance().stopIntaking();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    },
    INTAKING {
      @Override
      public void initialize() {
        getInstance().runIntake();
      }

      @Override
      public void execute() {
        getInstance().updateRollerSpeed();
      }

      @Override
      public SystemState nextState() {
        return s_requestedNextState;
      }
    }
  }

  public static void setState(IntakeStates nextState) {
    s_requestedNextState = nextState;
  }

  private static IntakeSubsystem s_intakeInstance;
  private static IntakeStates s_requestedNextState;

  private TalonFX m_intakeRollerLeader;
  private TalonFX m_intakeRollerFollower;
  private TalonFX m_armMotor;

  private VelocityVoltage m_intakeRollerRequest;
  private PositionVoltage m_armMotorRequest;

  public IntakeSubsystem() {
    super(IntakeStates.INACTIVE);
    setState(IntakeStates.INACTIVE);

    m_intakeRollerLeader = new TalonFX(Constants.Intake.LEADER_ROLLER_ID);
    m_intakeRollerFollower = new TalonFX(Constants.Intake.FOLLOWER_ROLLER_ID);
    m_armMotor = new TalonFX(Constants.Intake.ARM_MOTOR_ID);

    m_intakeRollerRequest = new VelocityVoltage(0);
    m_armMotorRequest = new PositionVoltage(0);

    m_intakeRollerFollower.setControl(
        new Follower(m_intakeRollerLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public static IntakeSubsystem getInstance() {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem();
    }
    return s_intakeInstance;
  }

  public static LinearVelocity getBallEntrySpeed() {
    SwerveDriveState robotState = DriveSubsystem.getDrivetrain().getState();
    ChassisSpeeds fieldRelativeSpeeds = robotState.Speeds;
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond,
            fieldRelativeSpeeds.omegaRadiansPerSecond,
            robotState.Pose.getRotation().unaryMinus());
    // don't go too slow or too fast
    return MetersPerSecond.of(
        Math.min(
            AimUtil.getBallVelocity().in(MetersPerSecond) / 2,
            Math.max(robotRelativeSpeeds.vxMetersPerSecond * 2, 2)));
  }

  private void runIntake() {
    updateRollerSpeed();
    m_armMotor.setControl(m_armMotorRequest.withPosition(Constants.Intake.ARM_DEPLOY_POSITION));
  }

  private void updateRollerSpeed() {
    AngularVelocity intakeVelocity =
        RotationsPerSecond.of(
            getBallEntrySpeed().in(MetersPerSecond)
                / (2 * Math.PI * Constants.Intake.OUTER_ROLLER_RADIUS.in(Meters)));
    m_intakeRollerLeader.setControl(m_intakeRollerRequest.withVelocity(intakeVelocity));
  }

  private void stopIntaking() {
    m_intakeRollerLeader.set(0);
    m_armMotor.setControl(m_armMotorRequest.withPosition(Constants.Intake.ARM_DEPLOY_POSITION));
  }

  private void stow() {
    m_intakeRollerLeader.set(0);
    m_armMotor.setControl(m_armMotorRequest.withPosition(Constants.Intake.ARM_STOW_POSITION));
  }
}
