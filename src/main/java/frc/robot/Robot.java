// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.serialization.SerializationSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private final CommandXboxController m_driverController;
  private boolean m_activeToggle;
  private boolean m_slowdownToggle;
  SendableChooser<String> autoChooser;
  Map<String, List<WaypointWithSpeed>> allAutos = new HashMap<>();

  public record WaypointWithSpeed(Pose2d waypoint, double velocity) {}
  ;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   *
   * @throws org.json.simple.parser.ParseException
   */
  public Robot() throws IOException, org.json.simple.parser.ParseException {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    DriveSubsystem.getInstance();
    IntakeSubsystem.getInstance();
    ShooterSubsystem.getInstance();
    SerializationSubsystem.getInstance();
    HeadHoncho.getInstance();
    AutoFollower.getInstance();
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_driverController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> m_activeToggle = !m_activeToggle));
    m_driverController.a().onTrue(Commands.runOnce(() -> m_slowdownToggle = !m_slowdownToggle));
    configureBindings();
  }

  @Override
  public void robotInit() {
    try {
      allAutos = new HashMap<>();
      for (String auto : AutoBuilder.getAllAutoNames()) {
        List<Double> maxVelocitiesPerAuto = new ArrayList<>();
        List<Pose2d> waypointsPerAuto = new ArrayList<>();
        List<Pose2d> waypointsPerAutoUnique = new ArrayList<>();
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(auto);
        for (int i = 0; i < paths.size(); i++) {
          PathPlannerPath path = paths.get(i);
          for (int j = 0; j < path.getWaypoints().size() - 1; j++) {
            maxVelocitiesPerAuto.add(path.getGlobalConstraints().maxVelocityMPS());
          }
          if (i == paths.size() - 1) {
            maxVelocitiesPerAuto.add(path.getGlobalConstraints().maxVelocityMPS());
          }
          List<Pose2d> waypointsPerPath = new ArrayList<>();
          for (Waypoint waypoint : path.getWaypoints()) {
            waypointsPerPath.add(new Pose2d(waypoint.anchor(), new Rotation2d(0)));
          }
          for (RotationTarget rotTarget : path.getRotationTargets()) {
            if (rotTarget.position() % 1 == 0) {
              waypointsPerPath.set(
                  (int) rotTarget.position(),
                  new Pose2d(
                      waypointsPerPath.get((int) rotTarget.position()).getTranslation(),
                      rotTarget.rotation()));
            }
          }
          waypointsPerPath.set(
              0,
              new Pose2d(
                  waypointsPerPath.get(0).getTranslation(),
                  path.getIdealStartingState().rotation()));
          waypointsPerPath.set(
              waypointsPerPath.size() - 1,
              new Pose2d(
                  waypointsPerPath.get(waypointsPerPath.size() - 1).getTranslation(),
                  path.getGoalEndState().rotation()));

          waypointsPerAuto.addAll(waypointsPerPath);
        }
        Pose2d prev = waypointsPerAuto.get(0);
        waypointsPerAutoUnique.add(prev);
        for (int j = 1; j < waypointsPerAuto.size(); j++) {
          Pose2d current = waypointsPerAuto.get(j);
          if (!current.getTranslation().equals(prev.getTranslation())) {
            waypointsPerAutoUnique.add(current);
            prev = current;
          }
        }
        List<WaypointWithSpeed> waypointsAndSpeeds = new ArrayList<>();
        int index = 0;
        for (double vel : maxVelocitiesPerAuto) {
          waypointsAndSpeeds.add(new WaypointWithSpeed(waypointsPerAutoUnique.get(index), vel));
          index++;
        }
        allAutos.put(auto, waypointsAndSpeeds);
      }
      System.out.println(allAutos);
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("None", null);
      List<String> autoNames = AutoBuilder.getAllAutoNames();
      for (String autoName : autoNames) {
        autoChooser.addOption(autoName, autoName);
      }
      SmartDashboard.putData("Auto Chooser", autoChooser);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  private void configureBindings() {
    HeadHoncho.getInstance()
        .configureBindings(() -> m_activeToggle, m_driverController.rightTrigger());
    DriveSubsystem.getInstance()
        .configureBindings(
            () -> m_driverController.getLeftY(), // drive x
            () -> m_driverController.getLeftX(), // drive y
            () -> m_driverController.getRightX(),
            () -> m_slowdownToggle);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    DriveSubsystem.getInstance().setPerspective();
    AutoFollower.setAuto(allAutos.get(autoChooser.getSelected()));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    DriveSubsystem.getInstance().setPerspective();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
