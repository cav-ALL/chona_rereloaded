// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveChassis extends SubsystemBase {

	// Gyroscope
	AHRS gyro = new AHRS(Port.kMXP);

	// Swerve Modules
	SwerveModule frontLeft = new SwerveModule(7, 8, 24, -0.063720703125, "Front Left", "rio");
	SwerveModule frontRight = new SwerveModule(5, 6, 21, 0.0849609375, "Front Right", "rio");
	SwerveModule backLeft = new SwerveModule(1, 2, 23, -0.433837890625, "Back Left", "rio");
	SwerveModule backRight = new SwerveModule(3, 4, 22, 0.2734375, "Back Right", "rio");

	// Swerve Kinematics
	SwerveDriveKinematics kinematics;

	// Field 2D
	Field2d field2d;

	// Swerve Odometry
	SwerveDrivePoseEstimator odometry;

	// Swerve Module Positions
	SwerveModulePosition[] modulePositions;

	// Desired Speeds
	ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	/** Creates a new SwerveChassis. */
	public SwerveChassis() {

		gyro.reset();

		kinematics = new SwerveDriveKinematics(Constants.SwerveChassisConstants.kModuleLocation);

		modulePositions = new SwerveModulePosition[] {
				frontLeft.getModulePosition(),
				frontRight.getModulePosition(),
				backLeft.getModulePosition(),
				backRight.getModulePosition()
		};

		odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), modulePositions, new Pose2d());

		field2d = new Field2d();

		AutoBuilder.configureHolonomic(
				this::getPose, // Robot pose supplier
				this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
													// Constants class
						new PIDConstants(2.55, 0.0, 0.0), // Translation PID constants
						new PIDConstants(0, 0.0, 0.0), // Rotation PID constants
						3.7, // Max module speed, in m/s
						0.459, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this // Reference to this subsystem to set requirements
		);

		SmartDashboard.putData("Chassis/Odometry", field2d);
	}

	// GetModulePosition
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				frontLeft.getModulePosition(),
				frontRight.getModulePosition(),
				backLeft.getModulePosition(),
				backRight.getModulePosition()
		};
	}

	// GetModuleStates
	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				frontLeft.getActualState(),
				frontRight.getActualState(),
				backLeft.getActualState(),
				backRight.getActualState()
		};
	}

	// Get the current pose of the robot
	public Pose2d getPose() {
		return odometry.getEstimatedPosition();
	}

	// Reset the odometry to a specified pose
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
	}

	// Reset odometry angle
	public void resetAngle(double angle) {
		Pose2d actualPose = getPose();
		Pose2d newPose = new Pose2d(actualPose.getTranslation(), Rotation2d.fromDegrees(angle));

		resetOdometry(newPose);
	}

	public Command resetHeading(double angle){
		return Commands.runOnce(() -> resetAngle(angle), this);
	}

	// Add vision measurements to the pose estimator
	public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
		odometry.addVisionMeasurement(visionPose, timestamp);
	}

	// Drive Robot Relative
	public void driveRobotRelative(ChassisSpeeds speeds) {
		desiredSpeeds = speeds;
	}

	// Drive Robot Field Relative
	public void driveFieldRelative(ChassisSpeeds speeds) {
		ChassisSpeeds tempSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.discretize(tempSpeeds, 0.20);

		driveRobotRelative(fieldRelativeSpeeds);
	}

	// Get robot relative speeds
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	// Set the desired states of the swerve modules
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		backLeft.setDesiredState(desiredStates[2]);
		backRight.setDesiredState(desiredStates[3]);
	}

	// Update Odometry
	public void updateOdometry() {
		odometry.update(gyro.getRotation2d(), getModulePositions());
	}

	public void updateShuffleboard() {

	}

	@Override
	public void periodic() {
		updateOdometry();

		SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(desiredSpeeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveModuleConstants.kMaxSpeed);

		setModuleStates(desiredStates);

		field2d.setRobotPose(odometry.getEstimatedPosition());

		SmartDashboard.putNumber("Position/X", odometry.getEstimatedPosition().getX());
		SmartDashboard.putNumber("Position/Y", odometry.getEstimatedPosition().getY());
		SmartDashboard.putNumber("Position/Rot", odometry.getEstimatedPosition().getRotation().getDegrees());

		updateShuffleboard();

	}
}
