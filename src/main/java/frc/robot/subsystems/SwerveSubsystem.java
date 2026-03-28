package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.RoboRIOSerialNumbers;
import frc.robot.Constants.TargetConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private Limelight limelight;
    private LimelightPoseEstimator poseEstimator;
    private final RobotContainer robotContainer;

    private PIDController aimingPID = new PIDController(DriveConstants.AIMING_PID_KP,DriveConstants.AIMING_PID_KI,DriveConstants.AIMING_PID_KD);

    private StructPublisher<Pose3d> robotPosePublisher;
    private StructPublisher<Pose3d> limelightPosePublisher;
    private StructPublisher<Pose3d> limelightOrbPosePublisher;

    public SwerveSubsystem(RobotContainer robotContainer) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            File swerveConfigDirectory = new File(Filesystem.getDeployDirectory(), getSwervePath());
            swerveDrive = new SwerveParser(swerveConfigDirectory).createSwerveDrive(DriveConstants.MAX_SPEED);
            if (Robot.isSimulation()) {
                swerveDrive.setHeadingCorrection(false);
                swerveDrive.setCosineCompensator(false);
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        try {
            limelight = new Limelight("limelight");
            limelight.getSettings()
                    .withLimelightLEDMode(LEDMode.PipelineControl)
                    .withCameraOffset(LimelightConstants.LIMELIGHT_POSITION)
                    .save();
            poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
        } catch (Exception e) {
            System.out.println("Failed to find limelight, Error: " + e.getMessage());
        }
        this.robotContainer = robotContainer;

        //Prepares publishes the robot pose publisher. The value is set in periodic.
        robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose3d.struct).publish();

        limelightPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Limelight Pose", Pose3d.struct).publish();

        limelightOrbPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Limelight Orb Pose", Pose3d.struct).publish();

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
                .withRobotOrientation(new Orientation3d(new Rotation3d(swerveDrive.getPose().getRotation()), new AngularVelocity3d(DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(swerveDrive.getGyro().getYawAngularVelocity().magnitude()))))
                .save();

            SmartDashboard.putNumber("Robot X", swerveDrive.getPose().getX());
            SmartDashboard.putNumber("Robot Y", swerveDrive.getPose().getY());

            //Setting the robots pose in network tables
            Pose3d robotPose = new Pose3d(getPose().getX(),getPose().getY(),0,new Rotation3d(getHeading()));
            
            robotPosePublisher.set(robotPose);
            limelightPosePublisher.set(getLimelightPose());
            limelightOrbPosePublisher.set(getLimelightOrbPose());

            SmartDashboard.putNumber("Heading Degrees", getHeading().getDegrees());

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
                if (poseEstimate.tagCount > 0 && poseEstimate.getMinTagAmbiguity() < 0.5) {
                    swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
                    swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
                    isVision = true;
                }
                SmartDashboard.putBoolean("isVision", isVision);
            });
        } catch (Exception e) { }

        //Outputting the distance from the hub on smartdashboard
        Pose2d hubPose = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) ? TargetConstants.RED_HUB_POSE3D.toPose2d() : TargetConstants.BLUE_HUB_POSE3D.toPose2d();
        double distanceFromHub = getPose().getTranslation().getDistance(hubPose.getTranslation());
        SmartDashboard.putNumber("Distance from Hub", distanceFromHub);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            if (robotContainer.shouldAim()) {
                aimAtHub(DriveConstants.AIMING_TOLERANCE, velocity);
            } else {
                swerveDrive.driveFieldOriented(velocity.get());
            }
        });
    }

    // Copied
    // https://github.com/Shenzhen-Robotics-Alliance/YAGSL-maple-sim/blob/main/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java#L69's
    // homework
    private void aimAtHub(double tolerance, Supplier<ChassisSpeeds> velocity) {
        SwerveController controller = swerveDrive.getSwerveController();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.get().vxMetersPerSecond,
                velocity.get().vyMetersPerSecond,
                controller.headingCalculate(getHeading().getRadians(),
                        getHubYaw().getRadians()),
                getHeading());
        
        SmartDashboard.putNumber("Hub Aim Error (degrees)", Math.abs(getHubYaw().minus(getHeading()).getDegrees()));
        if (Math.abs(getHubYaw().minus(getHeading()).getDegrees()) < tolerance)
            speeds.omegaRadiansPerSecond = 0;
        speeds.omegaRadiansPerSecond *= -1;
        speeds.omegaRadiansPerSecond *= 2;

        speeds.omegaRadiansPerSecond = aimingPID.calculate(velocity.get().omegaRadiansPerSecond,speeds.omegaRadiansPerSecond);
        
        SmartDashboard.putNumber("Hub angular velocity Rad", speeds.omegaRadiansPerSecond);

        drive(speeds);
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    private Rotation2d getHeading() {
        return getPose().getRotation();
        //return swerveDrive.getOdometryHeading();
        //return getLimelightPose().getRotation().toRotation2d();
    }

    private Rotation2d getHubYaw() {
        // Taken from PhotonUtils.getYawToPose()
        Translation2d relativeTrl = TargetConstants.BLUE_HUB_POSE3D.toPose2d().relativeTo(getPose()).getTranslation();
        return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getOdometryHeading());
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

    //This was ripped from last years code, so don't trust it.
    public void zeroGyroWithAlliance() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            swerveDrive.zeroGyro();
            swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            swerveDrive.zeroGyro();
        }
    }

    private Pose3d getLimelightPose(){
        double[] limelightData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[11]);
        return new Pose3d(limelightData[0],limelightData[1],limelightData[2],new Rotation3d(Math.toRadians(limelightData[3]),Math.toRadians(limelightData[4]),Math.toRadians(limelightData[5])));
    }

    private Pose3d getLimelightOrbPose(){
        double[] limelightData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_orb").getDoubleArray(new double[11]);
        return new Pose3d(limelightData[0],limelightData[1],limelightData[2],new Rotation3d(Math.toRadians(limelightData[3]),Math.toRadians(limelightData[4]),Math.toRadians(limelightData[5])));
    }
}
