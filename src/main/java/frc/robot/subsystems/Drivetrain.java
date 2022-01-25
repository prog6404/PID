/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.*;

import java.util.ArrayList;


public class Drivetrain extends SubsystemBase {

  private Spark driveLeftFront, driveLeftBack, driveRightBack, driveRightFront;
  private SpeedControllerGroup driveLeft, driveRight;
  private DifferentialDrive m_drive;
  private AHRS navX;
  private DigitalInput encoder_left;
  private boolean last;
  private int count = 0;
  
  public Drivetrain() {

    // Sensors
    navX = new AHRS();
    navX.reset();

    // Sparks
    driveLeftFront = new Spark(Motors.Ports.DRIVE_LEFT_FRONT);
    driveLeftBack = new Spark(Motors.Ports.DRIVE_LEFT_BACK);

    driveRightFront = new Spark(Motors.Ports.DRIVE_RIGHT_FRONT);
    driveRightBack = new Spark(Motors.Ports.DRIVE_RIGHT_BACK);

    // ControllerGroup
    driveLeft = new SpeedControllerGroup(driveLeftFront, driveLeftBack);
    driveRight = new SpeedControllerGroup(driveRightFront, driveRightBack);

    encoder_left = new DigitalInput(Sensors.Encoders.LEFT_ENCODER);
    last = encoder_left.get();

    // DifferentialDrive
    m_drive = new DifferentialDrive(driveLeft, driveRight);

    
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

    int esq = calculaVelocidades(-fwd, rot, false).get(0) > 0 ? 1 : -1;

    boolean temp = encoder_left.get();
    if(temp != last) count += esq;
    last = temp;



    SmartDashboard.putNumber("encoder count", count);
    SmartDashboard.putBoolean("Encoder left", temp);
    m_drive.arcadeDrive(-fwd, rot, false);
  }

  /**
   * stops the robot
   */
  public void stop(){
    m_drive.arcadeDrive(0, 0, false);
  }
  //#endregion
  

  private ArrayList<Double> calculaVelocidades (double xSpeed, double zRotation, boolean squareInputs){
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    ArrayList<Double> velocidades = new ArrayList<Double>();
    velocidades.add(leftMotorOutput);
    velocidades.add(rightMotorOutput);

    return velocidades;
  }

}
