// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.*;

import static frc.robot.Constants.OperatorInterfaceConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

   private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric robocentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /*Subsystems*/
  private final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();

  /*Autonomous*/
  private final SendableChooser<Command> autoChooser;
  private SendableChooser<Double> polarityChooser = new SendableChooser<>();

  /* Operator Interfaces, Real */
  private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  public RobotContainer() {

    drivetrainSubsys.configDrivetrainSubsys();
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Drive Polarity", polarityChooser);
    polarityChooser.setDefaultOption("Default", 1.0);
    polarityChooser.addOption("Positive", 1.0);
    polarityChooser.addOption("Negative", -1.0);

    
    configAutonomous();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public double drivlimiter;

  private void configDefaultCommands() {

    if (Utils.isSimulation()) {

      // drivetrainSubsys.setDefaultCommand(
      // drivetrainSubsys.applyRequest(() ->
      // drive.withVelocityX(-joystick.getRawAxis(0) * MaxSpeed) // Drive forward
      // // with negative Y
      // // (forward)
      // .withVelocityY(joystick.getRawAxis(1) * MaxSpeed) // Drive left with negative
      // X (left)
      // .withRotationalRate(-joystick2.getRawAxis(1) * MaxAngularRate)) // Drive
      // counterclockwise with negative X
      // // (left)
      // );
    } else {
      drivetrainSubsys.setDefaultCommand(
          drivetrainSubsys.applyRequest(
              () -> drive.withVelocityX(0.5 * polarityChooser.getSelected() * -xboxController.getLeftY() * MaxSpeed) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                  .withVelocityY(0.5 * polarityChooser.getSelected() * -xboxController.getLeftX() * MaxSpeed) // Drive
                                                                                                              // left
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
                  .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative
                                                                                // X (left)
          ));
    }
  }



  private void configAutonomous() {
    SmartDashboard.putData(autoChooser);
  }

  public void updatePose() {
    // puts the drivetrain pose on our dashboards
    SmartDashboard.putNumber("estimated drive pose x", drivetrainSubsys.getState().Pose.getX());
    SmartDashboard.putNumber("estimated drive pose y", drivetrainSubsys.getState().Pose.getY());
    SmartDashboard.putNumber("estimated drive pose rotation",
        drivetrainSubsys.getState().Pose.getRotation().getDegrees());




  }
}
