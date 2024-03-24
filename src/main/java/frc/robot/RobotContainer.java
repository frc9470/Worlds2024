// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;

import static frc.robot.Constants.ShooterConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private VisionSubsystem vision = new VisionSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
//        AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
//                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
//                        OperatorConstants.LEFT_Y_DEADBAND),
//                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
//                        OperatorConstants.LEFT_X_DEADBAND),
//                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
//                        OperatorConstants.RIGHT_X_DEADBAND),
//                driverXbox.getHID()::getYButtonPressed,
//                driverXbox.getHID()::getAButtonPressed,
//                driverXbox.getHID()::getXButtonPressed,
//                driverXbox.getHID()::getBButtonPressed);
//
//        // Applies deadbands and inverts controls because joysticks
//        // are back-right positive while robot
//        // controls are front-left positive
//        // left stick controls translation
//        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRightX(),
                () -> driverXbox.getRightY());


        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRawAxis(2));

        drivebase.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    }

    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.b().whileTrue(
                Commands.deferredProxy(() -> drivebase.driveToPose(
                        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                ));
        // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

        //button for intake
        driverXbox.leftBumper()
                .whileTrue(
                        intake.armToGround()
                                .andThen(intake.rollerIn())
                )
                .onFalse(
                        intake.armToTransfer()
                                .deadlineWith(intake.rollerIn())
                );

                // Speaker
        driverXbox.rightTrigger()
                .whileTrue(
                    shooter.runShooter(SHOOTER_RPM)
                            .alongWith(shooter.alignShooter(vision))
                            .alongWith(drivebase.alignToVision()) // or we define a command that does both with the vision
                            .andThen(shooter.runFeeder(FEEDER_SPEED)
                                    .withTimeout(1))
                            .andThen(shooter.stopShooter())
                );

        // Amp
        driverXbox.leftTrigger()
                .whileTrue(
                    shooter.armToAmp()
                            //.alongWith(drivebase.alignAmp())
                            .andThen(
                                    shooter.runShooter(AMP_RPM)
                                        .alongWith(shooter.runFeeder(FEEDER_SPEED))
                                        .withTimeout(1))
                            .andThen(shooter.stopShooter())
                );

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("mogged6962");
    }

    public void setDriveMode() {
        //drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
