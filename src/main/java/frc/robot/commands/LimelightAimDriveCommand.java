package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.LimelightHelpers;

public class LimelightAimDriveCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private double xSpeed;
    private double ySpeed;
    private double rot;
    private boolean fieldRelative;
    
    private double kP_Aim = 0.035;
    private double kP_Range = 0.1;
    private int pipelineIndex = 0;
    
    public LimelightAimDriveCommand(DrivetrainSubsystem drivetrain, 
                                    double xSpeed, double ySpeed, double rot, 
                                    boolean fieldRelative) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
        LimelightHelpers.setPipelineIndex("limelight", pipelineIndex);
    }
    
    @Override
    public void execute() {
        double finalXSpeed = xSpeed;
        double finalYSpeed = ySpeed;
        double finalRot = rot;
        boolean finalFieldRelative = fieldRelative;
        
        // Only use Limelight if we have a target
        if (LimelightHelpers.getTV("limelight")) {
            // Override rotation with Limelight aiming
            double tx = LimelightHelpers.getTX("limelight");
            double targetingAngularVelocity = tx * kP_Aim;
            targetingAngularVelocity *= drivetrain.MaxAngularRate;
            targetingAngularVelocity *= -1.0;
            finalRot = targetingAngularVelocity;
            
            // Override forward speed with Limelight ranging
            double ty = LimelightHelpers.getTY("limelight");
            double targetingForwardSpeed = ty * kP_Range;
            targetingForwardSpeed *= drivetrain.MaxSpeed;
            targetingForwardSpeed *= -1.0;
            finalXSpeed = targetingForwardSpeed;
            
            // While using Limelight, use robot-centric driving
            finalFieldRelative = false;
        }
        
        // Convert to chassis speeds and apply
        if (finalFieldRelative) {
            drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric()
                .withVelocityX(finalXSpeed)
                .withVelocityY(finalYSpeed)
                .withRotationalRate(finalRot));
        } else {
            drivetrain.setControl(drivetrain.robocentricDrive
                .withVelocityX(finalXSpeed)
                .withVelocityY(finalYSpeed)
                .withRotationalRate(finalRot));
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_ForceOff("limelight");
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run continuously
    }
}