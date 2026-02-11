package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import limelight.Limelight;
import limelight.networktables.LimelightSettings.LEDMode;

public class LimelightSubsytem {
    private final Limelight limelight;

    public LimelightSubsytem() {
        limelight = new Limelight("limelight");
        limelight.getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withCameraOffset(Pose3d.kZero)
                .save();
    }
}
