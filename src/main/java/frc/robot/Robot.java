// Copyright (c) FIRST and other WPILib contributors.`
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SimulationConstants;
import swervelib.SwerveDrive;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private final RobotContainer robotContainer;
	//private final Thread visionThread;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	public Robot() {
		robotContainer = new RobotContainer();

		//TODO: Check what data is logged by default, and add any fields you find to be necessarry.
		DataLogManager.start();

		if (isSimulation()) {
      		DriverStation.silenceJoystickConnectionWarning(true);
    	}

		// visionThread = new Thread(
		// 	() -> {
		// 		UsbCamera camera = CameraServer.startAutomaticCapture();
		// 		CvSink cvSink = CameraServer.getVideo();
		// 		CvSource outputStream = CameraServer.putVideo("Robot Camera", 640, 480);
		// 		Mat mat = new Mat();

		// 		camera.setResolution(640, 480);

		// 		while (!Thread.interrupted()) {
		// 			if (cvSink.grabFrame(mat) == 0) {
		// 				outputStream.notifyError(cvSink.getError());
		// 				continue;
		// 			}

		// 			Imgproc.rectangle(mat, new Point(320, 0), new Point(320, 480), new Scalar(255, 255, 255), 5);

		// 			outputStream.putFrame(mat);
		// 		}
		// 	});

		// visionThread.setDaemon(true);
		// visionThread.start();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		if(isSimulation()){
			if(autonomousCommand.getName().contains("Start Anywhere")){
				simPositionInit(robotContainer.getSwerveSubSystem().getSwerveDrive());
			}
		}

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		if (isSimulation()){
			simPositionInit(robotContainer.getSwerveSubSystem().getSwerveDrive());
		}
	}

	public void simPositionInit(SwerveDrive swerveDrive){

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            swerveDrive.zeroGyro();
            swerveDrive.resetOdometry(SimulationConstants.RED_SIM_STARTING_POSITION);
        } else {
            swerveDrive.zeroGyro();
            swerveDrive.resetOdometry(SimulationConstants.BLUE_SIM_STARTING_POSTIION);
        }
    }

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		SimulatedArena.overrideInstance(new Arena2026Rebuilt());
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		SimulatedArena.getInstance().simulationPeriodic();
	}
}
