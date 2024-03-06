// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.DriveConstants.*;

public class RotateToAngle extends Command {

  private SwerveDrive swerve; 
  private  PIDController rotateController; 
  
  private double start; 
  private double end; 
  
  private double angle;
  private double current;


  /** Creates a new RotateToAngle. */

  public RotateToAngle( double angle, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.angle = angle;
    this.swerve = swerve; 
    addRequirements(swerve);

    rotateController = new PIDController(
     
      0.1,
      0, 
      0.000
    );
    
    rotateController.reset();
    rotateController.setTolerance(2);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  rotateController.enableContinuousInput(0, 360);
   
  start = 0;

  end = start + angle;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotateController.setSetpoint(end);

    current = swerve.getGyro().getYaw();
    
    double rotationSpeed = rotateController.calculate(current, end);
    System.out.println("current: " + current + " end: " + end + " error: " + (end-current));
    System.out.println("rotation Speed: " + rotationSpeed);

    ChassisSpeeds radial = new ChassisSpeeds(0, 0, /*check this */rotationSpeed);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerve.getModuleStates(), MAX_ROTATE_SPEED);
   
    swerve.drive(radial, MAX_DRIVE_SPEED); 

    SmartDashboard.putNumber("Current",current);
    SmartDashboard.putNumber("Start", start);
    SmartDashboard.putNumber("end",end);
    SmartDashboard.putNumber("Rotation Speed", rotationSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
    swerve.stopModules();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    System.out.println("isFinished Reporting: " + rotateController.atSetpoint());
    
    return rotateController.atSetpoint();

  }
}
