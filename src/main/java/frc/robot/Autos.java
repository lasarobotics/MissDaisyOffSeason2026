package frc.robot;

public class Autos {
  public static record WAYPOINT(double x, double y, double maxSpeed, double rotation) {}

  public static Autos s_autoInstance;

  String auto;

  public String[] blueAutos = {
    "BLUE_LEFT_TRENCH", "BLUE_RIGHT_TRENCH", "BLUE_LEFT_BUMP", "BLUE_RIGHT_BUMP"
  };
  public String[] redAutos = {
    "RED_LEFT_TRENCH", "RED_RIGHT_TRENCH", "RED_LEFT_TRENCH", "RED_LEFT_TRENCH"
  };

  public Autos() {}

  public static Autos getInstance() {
    if (s_autoInstance == null) {
      s_autoInstance = new Autos();
    }
    return s_autoInstance;
  }

  public String[] getBlueAutos() {
    return blueAutos;
  }

  public String[] getRedAutos() {
    return blueAutos;
  }

  public void setAuto(String auto) {
    this.auto = auto;
  }

  public void startAuto() {}

  public void stopAuto() {}

  public void blueLeftTrench() {}

  public void blueRightTrench() {}

  public void blueLeftBump() {}

  public void blueRightBump() {}

  public void redLeftTrench() {}

  public void redRightTrench() {}

  public void redLeftBump() {}

  public void redRightBump() {}
}
