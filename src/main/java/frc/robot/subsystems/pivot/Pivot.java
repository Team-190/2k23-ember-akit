package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  public static final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(150);
  public static final Rotation2d IDLE_POSITION = Rotation2d.fromDegrees(150);
  public static final Rotation2d LAUNCH_POSITION = Rotation2d.fromDegrees(105.0);
  public static final Rotation2d DROP_POSITION = Rotation2d.fromDegrees(60.0);
  public static final Rotation2d AUTO_DROP_POSITION = Rotation2d.fromDegrees(45);
  public static final Rotation2d DEPLOYED_POSITION = Rotation2d.fromDegrees(0);

  public static final LoggedTunableNumber K_P = new LoggedTunableNumber("Pivot/kP", 0.2);
  public static final LoggedTunableNumber K_D = new LoggedTunableNumber("Pivot/kD", 0.001);
  public static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Pivot/maxVelocity", 600.0);
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Pivot/maxAcceleration", 650.0);

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          K_P.get(),
          0.0,
          K_D.get(),
          new TrapezoidProfile.Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));

  private boolean closedLoop = true;

  @AutoLogOutput private final Mechanism2d mechanism = new Mechanism2d(2.0, 2.0);
  private final MechanismRoot2d mechanismRoot = mechanism.getRoot("Root", 1.3, 0.1);
  private final MechanismLigament2d mechanismLigament =
      mechanismRoot.append(
          new MechanismLigament2d("Pivot", 0.4, 0.0, 4, new Color8Bit(Color.kLightGreen)));

  public Pivot(PivotIO io) {
    this.io = io;
    setDefaultCommand(run(() -> setDeployed(false)));
    resetPosition();
  }

  @AutoLogOutput
  public Rotation2d getPosition() {
    return inputs.positionRad;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    mechanismLigament.setAngle(getPosition());

    if (K_P.hasChanged(hashCode())) {
      controller.setP(K_P.get());
    }
    if (K_D.hasChanged(hashCode())) {
      controller.setD(K_D.get());
    }
    if (MAX_VELOCITY.hasChanged(hashCode()) || MAX_ACCELERATION.hasChanged(hashCode())) {
      controller.setConstraints(
          new TrapezoidProfile.Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    }

    if (DriverStation.isDisabled()) {
      controller.reset(getPosition().getDegrees());
    } else if (closedLoop) {
      double voltage = controller.calculate(getPosition().getDegrees());
      io.setVoltage(voltage);
    }
  }

  @AutoLogOutput
  public boolean atGoal() {
    return controller.getSetpoint().equals(controller.getGoal());
  }

  public void setDeployed(boolean deployed) {
    if (deployed) {
      controller.setGoal(DEPLOYED_POSITION.getDegrees());
    } else {
      controller.setGoal(IDLE_POSITION.getDegrees());
    }
  }

  public void setLaunch(boolean deployed) {
    if (deployed) {
      controller.setGoal(LAUNCH_POSITION.getDegrees());
    } else {
      controller.setGoal(IDLE_POSITION.getDegrees());
    }
  }

  public void setDrop(boolean deployed) {
    if (deployed) {
      controller.setGoal(DROP_POSITION.getDegrees());
    } else {
      controller.setGoal(IDLE_POSITION.getDegrees());
    }
  }

  public void setAutoDrop(boolean deployed) {
    if (deployed) {
      controller.setGoal(AUTO_DROP_POSITION.getDegrees());
    } else {
      controller.setGoal(IDLE_POSITION.getDegrees());
    }
  }

  public void resetPosition() {
    io.setPosition(STARTING_POSITION);
  }

  public Command deployCommand() {
    return startEnd(() -> setDeployed(true), () -> setDeployed(false));
  }

  public Command launchCommand() {
    return startEnd(() -> setLaunch(true), () -> setLaunch(false));
  }

  public Command dropCommand() {
    return startEnd(() -> setDrop(true), () -> setDrop(false));
  }

  public Command autoDropCommand() {
    return startEnd(() -> setAutoDrop(true), () -> setDrop(false));
  }

  public Command resetPositionCommand() {
    return runOnce(() -> resetPosition());
  }
}
