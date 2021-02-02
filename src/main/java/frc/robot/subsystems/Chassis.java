/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private TalonFX frontRight;
  private TalonFX backRight;
  private TalonFX frontLeft;
  private TalonFX backLeft;
  private PigeonIMU gyro;

  /**
   * Creates a new Chassis.
   */
  public Chassis() {
    frontLeft = new TalonFX(Constants.leftFront);
    backLeft = new TalonFX(Constants.leftBack);
    frontRight = new TalonFX(Constants.rightFront);
    backRight = new TalonFX(Constants.rightBack);
    gyro = new PigeonIMU(Constants.gyroPort);
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    frontRight.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    gyro.setFusedHeading(0);
  }

  public void setVelocity(double right,double left){
    frontLeft.set(ControlMode.Velocity, left);
    frontRight.set(ControlMode.Velocity, right);
  }

  public double getAngle(){
    double angle = gyro.getFusedHeading();
    if(angle<0){
      angle = -((-angle)%360.0);
      if(angle<-180){
        return 360.0+angle;
      }
      return angle;
    }
    angle = angle%360.0;
    if(angle>180){
      return angle-360.0;
    }
    return angle;
  }

  public int getRightPos(){
    return frontRight.getSelectedSensorPosition();
  }

  public int getLeftPos(){
    return frontLeft.getSelectedSensorPosition();
  }

  public double getRightVelocity(){
    return frontRight.getSelectedSensorVelocity()*10.0/Constants.pulsesPerMeter;
  }

  public double getLeftVelocity(){
    return frontLeft.getSelectedSensorVelocity()*10.0/Constants.pulsesPerMeter;
  }

  public void radialAccelaration(double velocity, double turns){
    if(Math.abs(velocity)<0.02){
      velocity = 0;
    }
    if(Math.abs(turns)<0.005){
      turns = 0;
    }
    velocity = velocity*Constants.maxVelocity;
    turns = turns*Constants.maxRadialAccelaration;
    double right = 0;
    double left = 0;
    if(velocity>0){
      if(turns>0){
        right = Math.sqrt(((velocity*velocity/turns)-(Constants.robotLength/2))*turns);
        left = Math.sqrt(((velocity*velocity/turns)+(Constants.robotLength/2))*turns);
      }
      else if(turns<0){
        right = Math.sqrt(((velocity*velocity/(-turns))+(Constants.robotLength/2))*(-turns));
        left = Math.sqrt(((velocity*velocity/(-turns))-(Constants.robotLength/2))*(-turns));
      }
      else{
        right = velocity;
        left = velocity;
      }
    }
    else if(velocity<0){
      if(turns>0){
        right = -Math.sqrt(((velocity*velocity/turns)-(Constants.robotLength/2))*turns);
        left = -Math.sqrt(((velocity*velocity/turns)+(Constants.robotLength/2))*turns);
      }
      else if(turns<0){
        right = -Math.sqrt(((velocity*velocity/(-turns))+(Constants.robotLength/2))*(-turns));
        left = -Math.sqrt(((velocity*velocity/(-turns))-(Constants.robotLength/2))*(-turns));
      }
      else{
        right = velocity;
        left = velocity;
      }
    }
    else{
      if(turns>0){
        right = -Math.sqrt(turns*Constants.robotLength/2);
        left = Math.sqrt(turns*Constants.robotLength/2);
      }
      else if(turns<0){
        right = Math.sqrt((-turns)*Constants.robotLength/2);
        left = -Math.sqrt((-turns)*Constants.robotLength/2);
      }
      else{
        right = velocity;
        left = velocity;
      }
    }
    setVelocity(right, left);
  }

  public void angularVelocity(double velocity, double turns){
    if(Math.abs(velocity)<0.02){
      velocity = 0;
    }
    if(Math.abs(turns)<0.005){
      turns = 0;
    }
    velocity = velocity*Constants.maxVelocity;
    turns = turns*Constants.maxAngularVelocity;
    double right = 0;
    double left = 0;
    if(velocity>0){
      if(turns>0){
        right = ((velocity/turns)-(Constants.robotLength/2))*turns;
        left = ((velocity/turns)+(Constants.robotLength/2))*turns;
      }
      else if(turns<0){
        right = ((velocity/(-turns))+(Constants.robotLength/2))*(-turns);
        left = ((velocity/(-turns))-(Constants.robotLength/2))*(-turns);
      }
      else{
        right = velocity;
        left = velocity;
      }
    }
    else if(velocity<0){
      if(turns>0){
        right = -((((-velocity)/turns)-(Constants.robotLength/2))*turns);
        left = -(((velocity/turns)+(Constants.robotLength/2))*turns);
      }
      else if(turns<0){
        right = -(((velocity/turns)+(Constants.robotLength/2))*(-turns));
        left = -(((velocity/turns)-(Constants.robotLength/2))*(-turns));
      }
      else{
        right = velocity;
        left = velocity;
      }
    }
    else{
      if(turns>0){
        right = -(turns*Constants.robotLength/2);
        left = (turns*Constants.robotLength/2);
      }
      else if(turns<0){
        right = -(turns*Constants.robotLength/2);
        left = (turns*Constants.robotLength/2);
      }
      else{
        right = velocity;
        left = velocity;
      }
    }
    setVelocity(right, left);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
