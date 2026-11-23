// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = Constants.Robot.MaxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric followRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final PIDController turnController = new PIDController(0.03, 0.0, 0.002);
    private final PIDController strafeController = new PIDController(0.05, 0.0, 0.002);
    private final PIDController driveController = new PIDController(0.25, 0.0, 0.01);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem();

    /** Updates the CTRE drivetrain pose estimator using the latest AprilTag measurement. */
    public void updateVisionOdometry() {
        for (EstimatedRobotPose est : vision.getEstimatedGlobalPoses()) {
            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds
            );
        }
    }

    /** Drives toward and centers on a specific AprilTag ID until it fills desiredAreaPercent of the frame. */
    public Command followAprilTagCommand(int tagId, double desiredAreaPercent) {
        turnController.setSetpoint(0.0);
        turnController.setTolerance(1.5);

        strafeController.setSetpoint(0.0);
        strafeController.setTolerance(1.5);

        driveController.setSetpoint(desiredAreaPercent);
        driveController.setTolerance(0.25); // Stop adjusting within +/- 0.25% frame area

        return drivetrain.applyRequest(() -> {
            var targetOpt = vision.getTargetWithId(tagId);

            if (targetOpt.isEmpty()) {
                return followRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
            }

            var detected = targetOpt.get();
            var target = detected.target();
            double yaw = target.getYaw();

            // Lateral strafe (VelocityY)
            double strafeSpeed = strafeController.calculate(yaw);
            if (detected.isRearCamera()) {
                strafeSpeed = -strafeSpeed;
            }
            strafeSpeed = MathUtil.clamp(strafeSpeed, -MaxSpeed * 0.25, MaxSpeed * 0.25);

            // Forward/backward (VelocityX) based on tag area
            double forwardSpeed = 0.0;
            if (!driveController.atSetpoint()) {
                forwardSpeed = driveController.calculate(target.getArea());
                if (detected.isRearCamera()) {
                    forwardSpeed = -forwardSpeed;
                }
                forwardSpeed = MathUtil.clamp(forwardSpeed, -0.8, 0.8);
            }

            double turnOutput = 0.0;

            return followRequest
                .withVelocityX(forwardSpeed)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(turnOutput);
        });
    }

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Hold X to follow AprilTag ID 7 until it reaches 4.0% frame area (change 7 to your test tag ID)
        joystick.x().toggleOnTrue(followAprilTagCommand(0, 4.0));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}