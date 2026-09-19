package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoFollower {
  private static AutoFollower s_autoFollower;
  private static List<Pose2d> selectedAuto;
  private static Pose2d currentStartPoint;
  private static Pose2d currentEndPoint;

  public static AutoFollower getInstance() {
    if (s_autoFollower == null) {
      s_autoFollower = new AutoFollower();
    }
    return s_autoFollower;
  }

  public static void setAuto(List<Pose2d> auto) {
    selectedAuto = auto;
    currentStartPoint = selectedAuto.get(0);
    currentEndPoint = selectedAuto.get(1);
    Logger.recordOutput("Auto", selectedAuto.toArray(new Pose2d[0]));
  }

  public static boolean checkCollision() {
    if (selectedAuto != null) {
      double startX = currentStartPoint.getX();
      double startY = currentStartPoint.getY();
      double endX = currentEndPoint.getX();
      double endY = currentEndPoint.getY();
      double robotX = DriveSubsystem.getInstance().getTranslation2d().getX();
      double robotY = DriveSubsystem.getInstance().getTranslation2d().getY();
      double radius = Inches.of(5).in(Meters);

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
      double sqrtDisc = Math.sqrt(discriminant);
      double t1 = (-b - sqrtDisc) / (2 * a);
      double t2 = (-b + sqrtDisc) / (2 * a);

      if (discriminant >= 0 && ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1))) return true;
      return false;
    } else {
      return false;
    }
  }
}
