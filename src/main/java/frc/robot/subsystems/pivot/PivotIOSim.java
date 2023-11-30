package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
  private final SingleJointedArmSim motorSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          20.0,
          0.5,
          0.5,
          Units.degreesToRadians(-20.0),
          Units.degreesToRadians(150.0),
          true,
          Units.degreesToRadians(150.0));
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.positionRad = Rotation2d.fromRadians(motorSim.getAngleRads());
    inputs.velocityRadPerSec = motorSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {motorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(Rotation2d position) {
    motorSim.setState(position.getRadians(), 0.0);
  }
}
