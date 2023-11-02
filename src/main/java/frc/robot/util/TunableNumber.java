package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber {
  private static final String tableKey = "TunableNumbers";

  private String key;
  private double defaultValue;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (Constants.tuningMode) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    } else {
      // SmartDashboard.delete(key);
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  public double get() {
    return Constants.tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }
}
