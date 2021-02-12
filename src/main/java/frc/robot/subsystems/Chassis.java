/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode; // import control modes
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // import the tlaonFX
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup; // import the speed control group type
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // import the diffrential drive
// some debugging power
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // import the base subsystem (which we extend)
import frc.robot.Constants; // import all the measured constants

public class Chassis extends SubsystemBase {

  // TO DO: check the engines direction, maybe invert
  private WPI_TalonFX frontRight; // front right engine
  private WPI_TalonFX frontLeft; // front left engine
  private WPI_TalonFX backRight; // back right engine
  private WPI_TalonFX backLeft; // back left engine
  private PigeonIMU gyro;
  private DifferentialDrive m_drive; // instance of the premade diffrential drive
  private SpeedControllerGroup leftMotors; // a group which contains both left motors
  private SpeedControllerGroup rightMotors; // a group which contains both right motors

  /**
   * Creates a new Chassis.
   */
  public Chassis() {
    frontLeft = new WPI_WPI_TalonFX(Constants.leftFront);
    backLeft = new WPI_WPI_TalonFX(Constants.leftBack);
    frontRight = new WPI_TalonFX(Constants.rightFront);
    backRight = new WPI_TalonFX(Constants.rightBack);
    gyro = new PigeonIMU(Constants.gyroPort);
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    frontRight.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    gyro.setFusedHeading(0);
  }

  public void setVelocity(double right, double left) {
    frontLeft.set(ControlMode.Velocity, left);
    frontRight.set(ControlMode.Velocity, right);
  }

  public double getAngle() {
    double angle = gyro.getFusedHeading();
    if (angle < 0) {
      angle = -((-angle) % 360.0);
      if (angle < -180) {
        return 360.0 + angle;
      }
      return angle;
    }
    angle = angle % 360.0;
    if (angle > 180) {
      return angle - 360.0;
    }
    return angle;
  }

  public int getRightPos() {
    return frontRight.getSelectedSensorPosition();
  }

  public int getLeftPos() {
    return frontLeft.getSelectedSensorPosition();
  }

  public double getRightVelocity() {
    return (
      frontRight.getSelectedSensorVelocity() * 10.0 / Constants.pulsesPerMeter
    );
  }

  public double getLeftVelocity() {
    return (
      frontLeft.getSelectedSensorVelocity() * 10.0 / Constants.pulsesPerMeter
    );
  }

  public void radialAccelaration(double velocity, double turns) {
    if (Math.abs(velocity) < 0.02) {
      velocity = 0;
    }
    if (Math.abs(turns) < 0.005) {
      turns = 0;
    }
    velocity = velocity * Constants.maxVelocity;
    turns = turns * Constants.maxRadialAccelaration;
    double right = 0;
    double left = 0;
    if (velocity > 0) {
      if (turns > 0) {
        right =
          Math.sqrt(
            ((velocity * velocity / turns) - (Constants.robotLength / 2)) *
            turns
          );
        left =
          Math.sqrt(
            ((velocity * velocity / turns) + (Constants.robotLength / 2)) *
            turns
          );
      } else if (turns < 0) {
        right =
          Math.sqrt(
            ((velocity * velocity / (-turns)) + (Constants.robotLength / 2)) *
            (-turns)
          );
        left =
          Math.sqrt(
            ((velocity * velocity / (-turns)) - (Constants.robotLength / 2)) *
            (-turns)
          );
      } else {
        right = velocity;
        left = velocity;
      }
    } else if (velocity < 0) {
      if (turns > 0) {
        right =
          -Math.sqrt(
            ((velocity * velocity / turns) - (Constants.robotLength / 2)) *
            turns
          );
        left =
          -Math.sqrt(
            ((velocity * velocity / turns) + (Constants.robotLength / 2)) *
            turns
          );
      } else if (turns < 0) {
        right =
          -Math.sqrt(
            ((velocity * velocity / (-turns)) + (Constants.robotLength / 2)) *
            (-turns)
          );
        left =
          -Math.sqrt(
            ((velocity * velocity / (-turns)) - (Constants.robotLength / 2)) *
            (-turns)
          );
      } else {
        right = velocity;
        left = velocity;
      }
    } else {
      if (turns > 0) {
        right = -Math.sqrt(turns * Constants.robotLength / 2);
        left = Math.sqrt(turns * Constants.robotLength / 2);
      } else if (turns < 0) {
        right = Math.sqrt((-turns) * Constants.robotLength / 2);
        left = -Math.sqrt((-turns) * Constants.robotLength / 2);
      } else {
        right = velocity;
        left = velocity;
      }
    }
    setVelocity(right, left);
  }

  public void angularVelocity(double velocity, double turns) {
    if (Math.abs(velocity) < 0.02) {
      velocity = 0;
    }
    if (Math.abs(turns) < 0.005) {
      turns = 0;
    }
    velocity = velocity * Constants.maxVelocity;
    turns = turns * Constants.maxAngularVelocity;
    double right = 0;
    double left = 0;
    if (velocity > 0) {
      if (turns > 0) {
        right = ((velocity / turns) - (Constants.robotLength / 2)) * turns;
        left = ((velocity / turns) + (Constants.robotLength / 2)) * turns;
      } else if (turns < 0) {
        right =
          ((velocity / (-turns)) + (Constants.robotLength / 2)) * (-turns);
        left = ((velocity / (-turns)) - (Constants.robotLength / 2)) * (-turns);
      } else {
        right = velocity;
        left = velocity;
      }
    } else if (velocity < 0) {
      if (turns > 0) {
        right =
          -((((-velocity) / turns) - (Constants.robotLength / 2)) * turns);
        left = -(((velocity / turns) + (Constants.robotLength / 2)) * turns);
      } else if (turns < 0) {
        right =
          -(((velocity / turns) + (Constants.robotLength / 2)) * (-turns));
        left = -(((velocity / turns) - (Constants.robotLength / 2)) * (-turns));
      } else {
        right = velocity;
        left = velocity;
      }
    } else {
      if (turns > 0) {
        right = -(turns * Constants.robotLength / 2);
        left = (turns * Constants.robotLength / 2);
      } else if (turns < 0) {
        right = -(turns * Constants.robotLength / 2);
        left = (turns * Constants.robotLength / 2);
      } else {
        right = velocity;
        left = velocity;
      }
    }
    setVelocity(right, left);
  }

  public void arcadeDrive(
    double xSpeed,
    double zRotation,
    boolean squareInputs
  ) {
    //     xSpeed - The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    // zRotation - The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    // squareInputs - If set, decreases the input sensitivity at low speeds.
    this.leftMotors = new SpeedControllerGroup((SpeedController)this.frontLeft, (SpeedController)this.backLeft);
    this.rightMotors = new SpeedControllerGroup((SpeedController)this.frontRight, (SpeedController)this.backRight);
    this.m_drive = new DifferentialDrive(this.leftMotors, this.rightMotors);
    this.m_drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  public void curvatureDrive(
    double xSpeed,
    double zRotation,
    boolean isQuickTurn
  ) {
    //     xSpeed - The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    // zRotation - The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    // isQuickTurn - If set, overrides constant-curvature turning for turn-in-place maneuvers.
    this.leftMotors = new SpeedControllerGroup((SpeedController)this.frontLeft, (SpeedController)this.backLeft);
    this.rightMotors = new SpeedControllerGroup((SpeedController)this.frontRight, (SpeedController)this.backRight);
    this.m_drive = new DifferentialDrive(this.leftMotors, this.rightMotors);
    this.m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initSendable(SendableBuilder builder) {}
}
