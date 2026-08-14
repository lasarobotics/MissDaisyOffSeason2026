// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends StateMachine {

  public enum DriveStates implements SystemState {
    AUTO {
      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) {
          return DRIVER_CONTROL;
        }
        return AUTO;
      }
    },
    DRIVER_CONTROL {

      @Override
      public void execute() {
        AngularVelocity rotationRate =
            Constants.DriveConstants.MAX_ANGULAR_RATE.times(
                -getInstance().m_rotateRequest.getAsDouble());

        s_drivetrain.setControl(
            s_drive
                .withVelocityX(
                    Constants.DriveConstants.MAX_SPEED.times(
                        -getInstance().m_strafeRequest.getAsDouble()
                            * Math.abs(getInstance().m_strafeRequest.getAsDouble())))
                .withVelocityY(
                    Constants.DriveConstants.MAX_SPEED.times(
                        -getInstance().m_driveRequest.getAsDouble()
                            * Math.abs(getInstance().getInstance().m_driveRequest.getAsDouble())))
                .withRotationalRate(rotationRate));
      }

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    },
    CLIMB_ALIGN {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public SystemState nextState() {
        return getInstance().m_selectedState;
      }
    }
  }

  private static DriveSubsystem s_driveInstance;
  private DoubleSupplier m_driveRequest;
  private DoubleSupplier m_strafeRequest;
  private DoubleSupplier m_rotateRequest;
  private DriveStates m_selectedState;
  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;

  public DriveSubsystem() {
    super(DriveStates.DRIVER_CONTROL);
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  public void configureBindings(
      DoubleSupplier strafeRequest, DoubleSupplier driveRequest, DoubleSupplier rotateRequest) {
    m_strafeRequest = strafeRequest;
    m_driveRequest = driveRequest;
    m_rotateRequest = rotateRequest;
  }

  public void setState(DriveStates state) {
    m_selectedState = state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
