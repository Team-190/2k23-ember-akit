package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class PivotIOTalonFX implements PivotIO {
  public static final double GEAR_RATIO = 200 / 11;

  private final TalonFX motor;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;

  public PivotIOTalonFX() {
    motor = new TalonFX(50);
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    // supply current limit
    configuration.CurrentLimits.SupplyCurrentLimit = 40.0;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // motor direction
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // set motor to brake mode
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // apply configuration to motor
    motor.getConfigurator().apply(configuration);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.position = Rotation2d.fromRotations(position.getValueAsDouble() / GEAR_RATIO);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPosition(Rotation2d position) {
    var configurator = motor.getConfigurator();
    configurator.setPosition(position.getRotations() * GEAR_RATIO);
  }
}
