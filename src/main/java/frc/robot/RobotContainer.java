// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LiftingArm;
import frc.robot.subsystems.SwerveChassis;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

	SwerveChassis chassis = new SwerveChassis();
	LiftingArm liftingArm = new LiftingArm();
	Intake intake = new Intake();
	Shooter shooter = new Shooter();
	

	CommandXboxController driverController = new CommandXboxController(0);
	CommandXboxController operatorController = new CommandXboxController(1);


	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("Intake", Commands.parallel(
				liftingArm.setAngle(Constants.LiftingArmConstants.OpenAngle),
				intake.setVoltage(Constants.IntakeConstants.GrabVolts)
			).withTimeout(3.5));

		NamedCommands.registerCommand("Close Intake", Commands.parallel(
				liftingArm.setAngle(Constants.LiftingArmConstants.ClosedAngle),
				intake.setVoltage(Constants.IntakeConstants.StopVolts)
			));

		NamedCommands.registerCommand("Shoot", Commands.sequence(
			shooter.setVelocity(Constants.ShooterConstants.ShootVelocity),
			intake.setVoltage(Constants.IntakeConstants.TrowVolts),
			Commands.waitSeconds(0.5),
			Commands.parallel(
				shooter.setVelocity((Constants.ShooterConstants.StopVelocity)),
				intake.setVoltage(Constants.IntakeConstants.StopVolts)
			)
		));


		autoChooser = AutoBuilder.buildAutoChooser();

		SmartDashboard.putData(autoChooser);

		configureBindings();
	}

	private void configureBindings() {
		chassis.setDefaultCommand(new Drive(chassis, driverController));

		operatorController.rightTrigger().whileTrue(shooter.setVelocity(Constants.ShooterConstants.ShootVelocity));
		operatorController.rightTrigger().onFalse(shooter.setVelocity(Constants.ShooterConstants.StopVelocity));

		operatorController.a().onTrue(liftingArm.setVoltage(Constants.LiftingArmConstants.ExtendVolts));
		operatorController.a().onFalse(liftingArm.setVoltage(Constants.LiftingArmConstants.StopVolts));

		operatorController.y().onTrue(liftingArm.setVoltage(Constants.LiftingArmConstants.RetractVolts));
		operatorController.y().onFalse(liftingArm.setVoltage(Constants.LiftingArmConstants.StopVolts));

		operatorController.b().whileTrue(liftingArm.setAngle(Constants.LiftingArmConstants.OpenAngle).alongWith((intake.setVoltage(Constants.IntakeConstants.GrabVolts))));
		operatorController.b().onFalse(liftingArm.setAngle(Constants.LiftingArmConstants.ClosedAngle).beforeStarting((intake.setVoltage(Constants.IntakeConstants.StopVolts))));

		driverController.rightBumper().onTrue(intake.setVoltage(Constants.IntakeConstants.GrabVolts));
		driverController.rightBumper().onFalse(intake.setVoltage(Constants.IntakeConstants.StopVolts));

		driverController.leftBumper().onTrue(intake.setVoltage(Constants.IntakeConstants.TrowVolts));
		driverController.leftBumper().onFalse(intake.setVoltage(Constants.IntakeConstants.StopVolts));

		driverController.start().onTrue(
			Commands.either(
				chassis.resetHeading(180), chassis.resetHeading(0), () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
		);

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
