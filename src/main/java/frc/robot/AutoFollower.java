package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoFollower {
  private static AutoFollower s_autoFollower;
  private static List<Pose2d> selectedAuto;
  private static int currentStartPointIndex;
  private static int currentEndPointIndex;

  public static AutoFollower getInstance() {
    if (s_autoFollower == null) {
      s_autoFollower = new AutoFollower();
    }
    return s_autoFollower;
  }

  public static void setAuto(List<Pose2d> auto) {
    selectedAuto = auto;
    currentStartPointIndex = 0;
    currentEndPointIndex = 1;
    if (selectedAuto != null) {
      Logger.recordOutput("Auto", selectedAuto.toArray(new Pose2d[0]));
    }
  }

  public static ChassisSpeeds getDesiredSpeeds() {
    if (selectedAuto != null) {
      Pose2d currentStartPoint = selectedAuto.get(currentStartPointIndex);
      Pose2d currentEndPoint = selectedAuto.get(currentEndPointIndex);
      double startX = currentStartPoint.getX();
      double startY = currentStartPoint.getY();
      double endX = currentEndPoint.getX();
      double endY = currentEndPoint.getY();
      double robotX = DriveSubsystem.getInstance().getTranslation2d().getX();
      double robotY = DriveSubsystem.getInstance().getTranslation2d().getY();
      double radius = Inches.of(5).in(Meters);
      Translation2d target;
      /*
       * We are essentially parameterizing x and y as functions of t, where 0 <= t <= 1
       * and the plugging x(t) and y(t) into the circle equation, and then simplifying,
       * to get a quadratic in the form stuffA * t^2 + stuffB * t + stuffC
       * and then we apply quadratic formula to a, b, and c (solving for t)
       * if discriminant is < 0, no solution exists
       * if disc. = 0, 1 solution
       * if disc > 0 2 solution
       * actual solutions for a, b, and c look like what is written below
       */
      double a = (endX - startX) * (endX - startX) + (endY - startY) * (endY - startY);
      double b =
          2 * (((endX - startX) * (startX - robotX)) + ((endY - startY) * (startY - robotY)));
      double c =
          (startX - robotX) * (startX - robotX)
              + (startY - robotY) * (startY - robotY)
              - (radius * radius);
      double discriminant = (b * b) - (4 * a * c);
      double t;
      if (discriminant < 0) {
        /*
         * find scalar projection of robot vector onto line vector, normalize, and thats our "t"
         */
        t =
            ((robotX * endX) + (robotY * endY))
                / (Math.hypot(robotX, robotY) * Math.hypot(endX, endY));
      } else if (discriminant == 0) {
        t = -b / (2 * a);
      } else {
        double sqrtDisc = Math.sqrt(discriminant);
        double t1 = (-b - sqrtDisc) / (2 * a);
        double t2 = (-b + sqrtDisc) / (2 * a);
        t = Math.max(t1, t2);
      }
      if (t > 1) {
        if (currentEndPointIndex < selectedAuto.size() - 1) {
          currentStartPointIndex = currentEndPointIndex;
          currentEndPointIndex += 1;
        }
        target = selectedAuto.get(currentStartPointIndex).getTranslation();
      } else if (t < 0) {
        target = currentStartPoint.getTranslation();
      } else {
        double xOfT = startX + t * (endX - startX);
        double yOfT = startX + t * (endX - startX);
        target = new Translation2d(xOfT, yOfT);
      }
      Translation2d delta = target.minus(new Translation2d(robotX, robotY));
      double deltaNorm = delta.getNorm();
      double xDesiredSpeed =
          TunerConstants.kSpeedAt12Volts.magnitude() * (delta.getX() / deltaNorm);
      double YDesiredSpeed =
          TunerConstants.kSpeedAt12Volts.magnitude() * (delta.getY() / deltaNorm);
      return new ChassisSpeeds(xDesiredSpeed, YDesiredSpeed, 0);
    }
    return new ChassisSpeeds(0, 0, 0);
  }
}
