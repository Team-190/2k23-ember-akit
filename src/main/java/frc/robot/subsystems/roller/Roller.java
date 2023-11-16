package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public static final int INTAKE_MOTOR_ID = 60;
  public static final double IDLE_SPEED = 0.08;
  public static final double INTAKE_SPEED = 0.5;
  public static final double DUMP_SPEED = -0.25;
  public static final double HIGH_LAUNCH_SPEED = -1.0;
  public static final double MID_LAUNCH_SPEED = -1.0;
  public static final double HAS_CUBE_VELOCITY_THRESHOLD = Math.PI;

  public Roller(RollerIO io) {
    this.io = io;
    setDefaultCommand(run(this::idleMotor));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  /**
   * Consumes the percent speed you want on a scale from -1 to 1, where negative is outtake and
   * positive is intake
   *
   * @param motorSpeed Percentage speed of the motor
   */
  public void setSpeedPercent(double motorSpeed) {
    io.setVoltage(motorSpeed * 12.0);
  }

  /** sets the intake speed to the predefined intake speed */
  public void intakeAtSetSpeed() {
    setSpeedPercent(INTAKE_SPEED);
  }

  /** drop cube into low hybrid node */
  public void dump() {
    setSpeedPercent(DUMP_SPEED);
  }
  /** launch cube into mid node */
  public void launchToMid() {
    setSpeedPercent(MID_LAUNCH_SPEED);
  }
  /** launch cube into high node */
  public void launchToHigh() {
    setSpeedPercent(HIGH_LAUNCH_SPEED);
  }

  /** Sets the power of the intake motor to zero */
  public void idleMotor() {
    setSpeedPercent(IDLE_SPEED);
  }

  public boolean hasCube() {
    return Math.abs(inputs.velocityRadPerSec) < HAS_CUBE_VELOCITY_THRESHOLD
        && inputs.appliedVolts > 0;
  }

  public Command dumpCommand() {
    return startEnd(() -> dump(), () -> idleMotor());
  }

  public Command intakeCommand() {
    return startEnd(() -> intakeAtSetSpeed(), () -> idleMotor());
  }

  public Command highLaunchCommand() {
    return startEnd(() -> launchToHigh(), () -> idleMotor());
  }

  public Command midLaunchCommand() {
    return startEnd(() -> launchToMid(), () -> idleMotor());
  }

  public Command intakeRumbleCommand(DoubleConsumer rumble) {
    return Commands.runEnd(
        () -> {
          if (hasCube()) rumble.accept(1.0);
        },
        () -> rumble.accept(0.0));
  }
}
