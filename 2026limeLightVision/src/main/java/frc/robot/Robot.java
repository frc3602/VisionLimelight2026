/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

 package frc.robot;

 
 import edu.wpi.first.wpilibj.TimedRobot;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.CommandScheduler;
 
 @SuppressWarnings("unused")
public class Robot extends TimedRobot {
 
   private final RobotContainer robotContainer = new RobotContainer();
 
 
   private Command autonomousCommand;
 
   @Override
   public void robotInit() {
    // robotContainer.startPose();
   }
 
   @Override
   public void robotPeriodic() {
     CommandScheduler.getInstance().run();
     // robotContainer.updatePose();
   }
 
   @Override
   public void disabledInit() {
   }
 
   @Override
   public void disabledPeriodic() {
   }
 
   @Override
   public void autonomousInit() {
     autonomousCommand = robotContainer.getAutonomousCommand();
     if (autonomousCommand != null) {
       autonomousCommand.schedule();
     }
     // robotContainer.resetSimulation();
   }
 
   @Override
   public void autonomousPeriodic() {
   }
 
   public void testInit() {
     CommandScheduler.getInstance().cancelAll();
   }
 
   @Override
   public void testPeriodic() {
   }
 
 }