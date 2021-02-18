/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode; // import control modes
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // import the tlaonFX
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup; // import the speed control group type
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // import the diffrential drive
// some debugging power
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // import the base subsystem (which we extend)
import frc.robot.Constants; // import all the measured constants
import frc.robot.commands.Drive.DriveStates;

public class Chassis extends SubsystemBase {

  // TO DO: check the engines direction, maybe invert
  private WPI_TalonSRX frontRight; // front right engine
  private WPI_TalonSRX frontLeft; // front left engine
  private WPI_TalonSRX backRight; // back right engine
  private WPI_TalonSRX backLeft; // back left engine
  private PigeonIMU gyro;
  private DifferentialDrive m_drive; // instance of the premade diffrential drive
  private SpeedControllerGroup leftMotors; // a group which contains both left motors
  private SpeedControllerGroup rightMotors; // a group which contains both right motors
  private double speed; 


  /**
   * Creates a new Chassis.
   */
  public Chassis(DriveStates dStates) {
    frontLeft = new WPI_TalonSRX(Constants.leftFront);
    backLeft = new WPI_TalonSRX(Constants.leftBack);
    frontRight = new WPI_TalonSRX(Constants.rightFront);
    backRight = new WPI_TalonSRX(Constants.rightBack);
    gyro = new PigeonIMU(Constants.gyroPort);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    frontRight.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    gyro.setFusedHeading(0);
    
    frontLeft.config_kP(0, Constants.kp); 
    frontLeft.config_kD(0, Constants.kd); 
    frontLeft.config_kI(0, Constants.ki); 
    frontRight.config_kP(0, Constants.kp); 
    frontRight.config_kD(0, Constants.kd); 
    frontRight.config_kI(0, Constants.ki); 
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    if (dStates == DriveStates.arcadeDrive || dStates == DriveStates.curvatureDrive) {
      this.leftMotors = new SpeedControllerGroup((SpeedController)this.frontLeft, (SpeedController)this.backLeft);
      this.rightMotors = new SpeedControllerGroup((SpeedController)this.frontRight, (SpeedController)this.backRight);
      this.m_drive = new DifferentialDrive(this.leftMotors, this.rightMotors);
      System.out.println("uaguicgsvcsilvuisvoshvuisfgvuhsuivasuvuihvuiesugvui");
      this.m_drive.setMaxOutput(0.5);
      m_drive.setRightSideInverted(false);
    }
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontRight.setInverted(true);
    backRight.setInverted(true);
  }

  public void setVelocity(double right, double left) {
    double leftPulseVelocity = left / Constants.pulsesPerMeter / 10; 
    double rightPulseVelocity = right / Constants.pulsesPerMeter / 10; 
    // frontLeft.set(ControlMode.Velocity, left);
    // frontRight.set(ControlMode.Velocity, right);
    if (right == 0) {
      frontRight.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward,
    0);
    }
    else {
      frontRight.set(ControlMode.Velocity, rightPulseVelocity, DemandType.ArbitraryFeedForward,
    (Math.signum(right) * Constants.ks + Constants.kv * right) / 12);
    }
    if (left == 0) {
      frontLeft.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward,
    0);
    }
    else {
      frontLeft.set(ControlMode.Velocity, leftPulseVelocity, DemandType.ArbitraryFeedForward,
    (Math.signum(left) * Constants.ks + Constants.kv * left) / 12);
    }
    SmartDashboard.putNumber("leftVel", this.getLeftVelocity());
    SmartDashboard.putNumber("rightVel", this.getRightVelocity()); 
    SmartDashboard.putNumber("wantedLeftVel", left); 
    SmartDashboard.putNumber("wantedRightVel", right); 
  }

  public double getAngle() {
    double angle = gyro.getFusedHeading();

    System.out.println("uaguicgsvcsilvuisvoshvuisfgvuhsuivasuvuihvuiesugvui");
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
    if (Math.abs(turns) < 0.02) {
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
    if (Math.abs(turns) < 0.02) {
      turns = 0;
    }
    velocity = velocity * Constants.maxVelocity;
    turns = turns * Constants.maxAngularVelocity;
    System.out.println("velocity is: " + velocity);
    SmartDashboard.putNumber("velocity", velocity); 
    SmartDashboard.putNumber("Turns", turns); 
    double right = 0.5;
    double left = 0.5;
    if (velocity > 0) {
      if (turns > 0) {
        right = ((velocity / turns) - (Constants.robotLength / 2)) * turns;
        left = ((velocity / turns) + (Constants.robotLength / 2)) * turns;
      } else if (turns < 0) {
        right = ((velocity / (-turns)) + (Constants.robotLength / 2)) * (-turns);
        left = ((velocity / (-turns)) - (Constants.robotLength / 2)) * (-turns);
      } else {
        right = velocity;
        left = velocity;
      }
    } else if (velocity < 0) {
      if (turns > 0) {
        right = -((((-velocity) / turns) - (Constants.robotLength / 2)) * turns);
        left = -((((-velocity) / turns) + (Constants.robotLength / 2)) * turns);
      } else if (turns < 0) {
        right = -(((velocity / turns) + (Constants.robotLength / 2)) * (-turns));
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
    SmartDashboard.putNumber("left", left); 
    SmartDashboard.putNumber("right", right); 
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
    
    this.m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty(key, getter, setter);


  }
}
