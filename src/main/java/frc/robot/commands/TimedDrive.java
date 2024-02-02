// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;


public class TimedDrive extends Command {
  /** Creates a new DriveFoward. */

  private final Timer m_timer = new Timer();
  private SwerveDrive swerve;
  private double driveTime;
  private double xSpeed;
  private double ySpeed;
  
  public TimedDrive( SwerveDrive swerve, double driveTime, double xSpeed, double ySpeed ) {
    // Use addRequirements() here to declare subsystem dependencies.
    
      this.swerve = swerve; 
      this.driveTime = driveTime; 
      this.xSpeed = xSpeed;
      this.ySpeed = ySpeed;
    

      addRequirements(swerve);



  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_timer.restart();
    System.out.println("Initializing Timer");

  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

    System.out.println("Timer: " + m_timer.get() + " ");
    swerve.drive(xSpeed, ySpeed, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > driveTime;
  }
}