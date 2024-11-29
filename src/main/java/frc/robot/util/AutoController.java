package frc.robot.util;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BiConsumer;

public class AutoController implements BiConsumer<Pose2d, SwerveSample> {
  private final Drive drive;

  private final LoggedTunableNumber xkP = new LoggedTunableNumber("Auto/Tunables/xkP");
  private final LoggedTunableNumber xkD = new LoggedTunableNumber("Auto/Tunables/xkD");
  private final LoggedTunableNumber ykP = new LoggedTunableNumber("Auto/Tunables/ykP");
  private final LoggedTunableNumber ykD = new LoggedTunableNumber("Auto/Tunables/ykD");
  private final LoggedTunableNumber rkP = new LoggedTunableNumber("Auto/Tunables/rkP");
  private final LoggedTunableNumber rkD = new LoggedTunableNumber("Auto/Tunables/rkD");

  private final PIDController xController = new PIDController(xkP.get(), 0.0, xkD.get());
  private final PIDController yController = new PIDController(ykP.get(), 0.0, ykD.get());
  private final PIDController headingController = new PIDController(rkP.get(), 0.0, rkD.get());

  public AutoController(Drive drive) {
    this.drive = drive;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void accept(Pose2d pose, SwerveSample referenceState) {
    updateTunables();

    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;

    double xFeedback = xController.calculate(pose.getX(), referenceState.x);
    double yFeedback = yController.calculate(pose.getY(), referenceState.y);
    double rotationFeedback =
        headingController.calculate(pose.getRotation().getRadians(), referenceState.heading);

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());

    drive.runVelocity(out);
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> xController.setPID(xkP.get(), 0, xkD.get()), xkP, xkD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> yController.setPID(ykP.get(), 0, ykD.get()), ykP, ykD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> headingController.setPID(rkP.get(), 0, rkD.get()), rkP, rkD);
  }
}
