package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;
import frc.robot.util.Alert;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PoseManager;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
  private String name;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private double lastTimestamp = 0;

  private final double DEFAUlT_CROP = 0.9;
  private final double CROP_BUFFER = 0.1;

  public AprilTagVisionIOLimelight(String camName) {
    name = camName;

    LimelightHelpers.setLEDMode_PipelineControl(name);

    disconnectedAlert = new Alert("No data from: " + name, Alert.AlertType.ERROR);

    resetCropping();
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs, PoseManager poseManager) {
    LimelightHelpers.SetRobotOrientation(
        "limelight", poseManager.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate observation =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    inputs.estimatedPose = observation.pose;
    inputs.timestamp = observation.timestampSeconds;
    inputs.tagCount = observation.tagCount;
    inputs.avgTagDist = observation.avgTagDist;
    inputs.avgTagArea = observation.avgTagArea;

    inputs.pipeline = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.ledMode = LimelightHelpers.getLimelightNTDouble(name, "ledMode");

    // Update disconnected alert
    if (observation.timestampSeconds != 0) {
      lastTimestamp = observation.timestampSeconds;
    }
    disconnectedAlert.set(Timer.getFPGATimestamp() - lastTimestamp < disconnectedTimeout);

    dynamicCropping();
  }

  @Override
  public void setPipeline(int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(name, pipelineIndex);
  }

  @Override
  public void setPipeline(Pipelines pipelineEnum) {
    LimelightHelpers.setPipelineIndex(name, Pipelines.getIndexFor(pipelineEnum));
  }

  private void dynamicCropping() {
    double[] tcornxy = LimelightHelpers.getLimelightNTDoubleArray(name, "tcornxy");
    if (tcornxy.length == 0) {
      resetCropping();
      return;
    }

    double minX = tcornxy[0];
    double maxX = tcornxy[0];
    double minY = tcornxy[1];
    double maxY = tcornxy[1];

    // Iterate over all tag corners
    if (tcornxy.length > 2) {
      for (int i = 2; i < tcornxy.length - 1; i += 2) {
        minX = Math.min(minX, tcornxy[i]);
        maxX = Math.max(maxX, tcornxy[i]);
      }
      for (int i = 3; i < tcornxy.length; i += 2) {
        minY = Math.min(minY, tcornxy[i]);
        maxY = Math.max(maxY, tcornxy[i]);
      }
    }

    // Apply crop buffer and clamp to default crop size
    double cropXMin = MathUtil.clamp(minX - CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
    double cropXMax = MathUtil.clamp(maxX + CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
    double cropYMin = MathUtil.clamp(minY - CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
    double cropYMax = MathUtil.clamp(maxY + CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);

    LimelightHelpers.setCropWindow(name, cropXMin, cropXMax, cropYMin, cropYMax);
  }

  private void resetCropping() {
    LimelightHelpers.setCropWindow(name, -DEFAUlT_CROP, DEFAUlT_CROP, -DEFAUlT_CROP, DEFAUlT_CROP);
  }
}
