/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class Drivetrain extends PIDSubsystem {

  //#region Sensors, controllers & etc.
  private Spark driveLeftFront, driveLeftBack, driveRightBack, driveRightFront;
  private SpeedControllerGroup driveLeft, driveRight;
  private DifferentialDrive m_drive;
  private AHRS navX;

  //SHUFFLEBOARD
  private ShuffleboardTab drivetrainTab;
  //#endregion
  
  public Drivetrain() {
    super(new PIDController(0,0,0));

    // Sensors
    navX = new AHRS();

    // Sparks
    driveLeftFront = new Spark(Motors.Ports.DRIVE_LEFT_FRONT);
    driveLeftBack = new Spark(Motors.Ports.DRIVE_LEFT_BACK);

    driveRightFront = new Spark(Motors.Ports.DRIVE_RIGHT_FRONT);
    driveRightBack = new Spark(Motors.Ports.DRIVE_RIGHT_BACK);

    // ControllerGroup
    driveLeft = new SpeedControllerGroup(driveLeftFront, driveLeftBack);
    driveRight = new SpeedControllerGroup(driveRightFront, driveRightBack);

    // DifferentialDrive
    m_drive = new DifferentialDrive(driveLeft, driveRight);

    //Shuffle
    drivetrainTab = Shuffleboard.getTab("Tração");
  }

  @Override
  public void periodic() {
  }

  //#region ARCADE DRIVE

  /**
   * drives the robot based in fwd (forward) and
   * @param fwd value from 1 to 0 for the robot to move forward
   * @param rot value from 1 to 0 for the robot to rotate the robot
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(-fwd, rot);
  }

  /**
   * stops the robot
   */
  public void stop(){
    m_drive.arcadeDrive(0, 0);
  }
  //#endregion

  @Override
  protected void useOutput(double output, double setpoint) {
    Shuffleboard.getTab("DriveTrain").addNumber("PID Output", () -> output);
  }

  @Override
  protected double getMeasurement() {
    return 0;
  }

  /**
   * Gets the value of the heading of the robot
   * @return the value of the heading of the robot
   */
  public double getHeading() {
    return navX.getAngle();//Math.IEEEremainder(_navX.getYaw(), 360); //* -1.0d;
  }
  
  public void resetNavx() {
    navX.reset();
  }
}
