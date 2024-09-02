// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveChassis;

public class Drive extends Command {

	CommandXboxController controller;
	SwerveChassis chassis;

	int alliance = 1;

	SlewRateLimiter xLimiter = new SlewRateLimiter(10);
	SlewRateLimiter yLimiter = new SlewRateLimiter(10);
	SlewRateLimiter rotLimiter = new SlewRateLimiter(Units.rotationsToRadians(3));

	/** Creates a new Drive. */
	public Drive(SwerveChassis chassis, CommandXboxController controller) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.chassis = chassis;
		this.controller = controller;

		addRequirements(chassis);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			alliance = -1;
		} else {
			alliance = 1;
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double xInput = MathUtil.applyDeadband(-controller.getLeftY(), 0.2) * Constants.SwerveModuleConstants.kMaxSpeed;
		double yInput = MathUtil.applyDeadband(-controller.getLeftX(), 0.2) * Constants.SwerveModuleConstants.kMaxSpeed;
		double rotInput = MathUtil.applyDeadband(-controller.getRightX(), 0.2)
				* Constants.SwerveModuleConstants.KMaxAngularSpeed;

		// SmartDashboard.putNumber("Sticks/X", xInput);
		// SmartDashboard.putNumber("Sticks/Y", yInput);
		// SmartDashboard.putNumber("Sticks/R", rotInput);

		if (controller.getHID().getRightBumper()) {
			xInput = xLimiter.calculate(xInput);
			yInput = yLimiter.calculate(yInput);
			rotInput = rotLimiter.calculate(rotInput);

			chassis.driveRobotRelative(new ChassisSpeeds(xInput, yInput, rotInput));
		} else {
			xInput = xLimiter.calculate(xInput * alliance);
			yInput = yLimiter.calculate(yInput * alliance);
			rotInput = rotLimiter.calculate(rotInput);

			chassis.driveFieldRelative(new ChassisSpeeds(xInput, yInput, rotInput));
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
