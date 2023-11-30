package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d positionRad = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(Rotation2d position) {}
}
