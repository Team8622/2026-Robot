package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RoboRIOSerialNumbers;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private Limelight limelight;
    private LimelightPoseEstimator poseEstimator;
    private final RobotContainer robotContainer;

    public SwerveSubsystem(RobotContainer robotContainer) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            File swerveConfigDirectory = new File(Filesystem.getDeployDirectory(), getSwervePath());
            swerveDrive = new SwerveParser(swerveConfigDirectory).createSwerveDrive(DriveConstants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        try {
            limelight = new Limelight("limelight");
            limelight.getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withCameraOffset(new Pose3d(0.34925, -0.2159, 0.3683, new Rotation3d(0, 0.436332313, 0)))
                .save();
                poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
        } catch (Exception e) {
            System.out.println("Failed to find limelight, Error: " + e.getMessage());
        }
        this.robotContainer = robotContainer;

        SmartDashboard.putNumber("Hub X", DriverStation.getAlliance().equals(Alliance.Red) ? 0 : 0);
        SmartDashboard.putNumber("Hub Y", 2.91846);
        SmartDashboard.putNumber("velocity_test_input", 0.0);

        setupPathPlanner();
    }

    private void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> swerveDrive.drive(speeds), // Method that will drive the robot given ROBOT
                                                                     // RELATIVE ChassisSpeeds. Also optionally
                                                                     // outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                // for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> { // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner: " + e, e.getStackTrace());
        }
    }

    private String getSwervePath() {
        String subDirectory = "comp_chassis";
        String serialNumber = System.getenv("serialnum");

        // Determine chassis configuration based on the "serialNumber" environment
        // variable. If not set it defaults to comp_chassis.
        switch (serialNumber != null ? serialNumber : "") {
            case RoboRIOSerialNumbers.COMP_CHASSIS:
                subDirectory = "comp_chassis";
                break;
            case RoboRIOSerialNumbers.TEST_CHASSIS:
                subDirectory = "test_chassis";
                break;
        }

        return "swerve/" + subDirectory;
    }

    @Override
    public void periodic() {
        try {
            limelight.getSettings()
                .withRobotOrientation(new Orientation3d(swerveDrive.getGyroRotation3d(), new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
                .save();

            SmartDashboard.putNumber("Robot X", swerveDrive.getPose().getX());
            SmartDashboard.putNumber("Robot Y", swerveDrive.getPose().getY());

            double[] limelightIMUData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("imu").getDoubleArray(new double[]{0.0});
            double robotRotation = limelightIMUData[0] * (Math.PI / 180) - Math.PI;
            SmartDashboard.putNumber("Robot Rotation Degrees", robotRotation * (180/Math.PI));

            Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
            visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
                // If the average tag distance is less than 4 meters,
                // there are more than 0 tags in view,
                // and the average ambiguity between tags is less than 30% then we update the
                // pose estimation.
                SmartDashboard.putNumber("Avg Tag Dist", poseEstimate.avgTagDist);
                SmartDashboard.putNumber("Tag Count", poseEstimate.tagCount);
                SmartDashboard.putNumber("Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());

                boolean isVision = false;
                if (poseEstimate.avgTagDist < 4 && poseEstimate.tagCount > 0 && poseEstimate.getMinTagAmbiguity() < 0.3) {
                    swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
                    isVision = true;
                }
                SmartDashboard.putBoolean("isVision", isVision);
            });
        } catch (Exception e) { }
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            if (robotContainer.shouldAim()) {
                swerveDrive.drive(new Translation2d(velocity.get().vxMetersPerSecond, velocity.get().vyMetersPerSecond), getTargetVelocity(), false, false);
            } else {
                swerveDrive.driveFieldOriented(velocity.get());
            }
        });
    }

    private double getTargetVelocity() {
        double kP = .045;

        double robotX = swerveDrive.getPose().getX();
        double robotY = swerveDrive.getPose().getY();

        double[] limelightIMUData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("imu").getDoubleArray(new double[]{0.0});
        double robotRotation = 0.0;
        robotRotation = limelightIMUData[0] * (Math.PI / 180) - Math.PI;

        double hubX = SmartDashboard.getNumber("Hub X", 0.0);
        double hubY = SmartDashboard.getNumber("Hub Y", 0.0);

        double targetAngle = Math.atan2(hubY - robotY, hubX - robotX);
        SmartDashboard.putNumber("Target Angle", targetAngle);
        
        //double targetingAngularVelocity = Math.atan2(Math.sin(targetAngle - robotRotation), Math.cos(targetAngle - robotRotation));
        double targetingAngularVelocity = (targetAngle - robotRotation + Math.PI) % (2 * Math.PI) - Math.PI;
        //targetingAngularVelocity = targetingAngularVelocity > 0 ? targetingAngularVelocity : -Math.PI + targetingAngularVelocity;
        targetingAngularVelocity *= kP * swerveDrive.getMaximumChassisAngularVelocity();

        //Current 60 Target 30
        // 30 - 60 = -30
        // Current 60 Target -30
        // (180 - -60) % 180 = 60.

        SmartDashboard.putNumber("Robot Rotation", robotRotation);
        SmartDashboard.putNumber("Target Angular Velocity", targetingAngularVelocity);

        return SmartDashboard.getNumber("velocity_test_input", 0.0);
    }

    // Required for PathPlanner
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    // Required for PathPlanner
    public void resetPose(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    // Required for PathPlanner
    public ChassisSpeeds getCurrentSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
