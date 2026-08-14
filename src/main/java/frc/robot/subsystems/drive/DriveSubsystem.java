// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.generated.TunerConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

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
                    Constants.DriveConstants.MAX_SPEED
                        .times(
                            -getInstance().m_strafeRequest.getAsDouble()
                                * Math.abs(getInstance().m_strafeRequest.getAsDouble()))
                        .times(getInstance().m_currentSpeedScalar))
                .withVelocityY(
                    Constants.DriveConstants.MAX_SPEED
                        .times(
                            -getInstance().m_driveRequest.getAsDouble()
                                * Math.abs(getInstance().m_driveRequest.getAsDouble()))
                        .times(getInstance().m_currentSpeedScalar))
                .withRotationalRate(rotationRate.times(getInstance().m_currentSpeedScalar)));
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
  private BooleanSupplier m_slowdownRequest;
  private double m_currentSpeedScalar;

  // private PIDController m_rotationPIDController;

  public DriveSubsystem() {
    super(DriveStates.DRIVER_CONTROL);
    s_drivetrain = TunerConstants.createDrivetrain();
    setPerspective();
    s_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(
                Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    // m_rotationPIDController =
    //     new PIDController(
    //         Constants.DriveConstants.TURN_P,
    //         Constants.DriveConstants.TURN_I,
    //         Constants.DriveConstants.TURN_D);
    // m_rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setPerspective() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      }
      if (ally.get() == Alliance.Blue) {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
    }
  }

  public static DriveSubsystem getInstance() {
    if (s_driveInstance == null) {
      s_driveInstance = new DriveSubsystem();
    }
    return s_driveInstance;
  }

  public void configureBindings(
      DoubleSupplier strafeRequest,
      DoubleSupplier driveRequest,
      DoubleSupplier rotateRequest,
      BooleanSupplier slowdownRequest) {
    m_strafeRequest = strafeRequest;
    m_driveRequest = driveRequest;
    m_rotateRequest = rotateRequest;
    m_slowdownRequest = slowdownRequest;
  }

  public void setState(DriveStates state) {
    m_selectedState = state;
  }

  @Override
  public void periodic() {
    m_currentSpeedScalar =
        m_slowdownRequest.getAsBoolean() ? Constants.DriveConstants.SLOWDOWN_SPEED : 1;
    Logger.recordOutput(getName() + "/Pose", s_drivetrain.getState().Pose);
    // This method will be called once per scheduler run
  }
}
