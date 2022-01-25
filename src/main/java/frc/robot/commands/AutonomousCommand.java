// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonomousCommand extends CommandBase {
  
  private Drivetrain m_driveTrain;
  private AHRS navX;
  private double setpoint;
  private PIDController pidControl;

  public AutonomousCommand(Drivetrain drivetrain) {
    m_driveTrain = drivetrain;
    navX = new AHRS();
    pidControl = new PIDController(0.01, 0, 0);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = navX.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    SmartDashboard.putNumber("Setpoint Error", setpoint - navX.getAngle());  
    SmartDashboard.putNumber("PID out", pidControl.calculate(navX.getAngle()));
    m_driveTrain.arcadeDrive(0, pidControl.calculate(navX.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
