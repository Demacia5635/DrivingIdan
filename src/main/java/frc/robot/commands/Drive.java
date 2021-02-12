/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import java.lang.Math.toDegrees; 
import java.lang.Math.atan; 


public class Drive extends CommandBase {

  public enum InputHandler{
    tank,singer,triggerTurns,YandX, arcadeDrive, curvatureDrive
  }
  public enum driveStates {
    curvatureDrive, arcadeDrive, angularVelocity, radialAccelaration
  }
  private final Chassis chassis;
  private XboxController controller;
  private InputHandler inputHandler;
  private driveStates driveStates; 
  /**
   * Creates a new Drive.
   */
  public Drive(Chassis chassis, InputHandler inputHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    controller = new XboxController(Constants.xboxPort);
    this.inputHandler = inputHandler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = 0;
    double turns = 0;
    switch(inputHandler){
      case YandX:
        velocity = controller.getY(Hand.kLeft);
        turns = controller.getX(Hand.kRight);
        chassis.radialAccelaration(velocity, turns);
        break;
      case tank:
        chassis.setVelocity(controller.getY(Hand.kRight), controller.getY(Hand.kLeft));
        break;
      case singer:
        velocity = controller.getTriggerAxis(Hand.kRight)-controller.getTriggerAxis(Hand.kLeft);
        turns = controller.getX(Hand.kLeft);
        chassis.radialAccelaration(velocity, turns);
        break;
      case triggerTurns:
        velocity = controller.getY(Hand.kLeft);
        turns = controller.getTriggerAxis(Hand.kRight)-controller.getTriggerAxis(Hand.kLeft);
        chassis.radialAccelaration(velocity, turns);
        break;
    }
    double zRotation = -5, xSpeed = 5; 
    switch(driveStates) {
      case arcadeDrive: 
        zRotation = Math.toDegrees(Math.atan(controller.getY(Hand.kRight)/controller.getX(Hand.kRight))) / 180; 
        xSpeed = controller.getY(Hand.kLeft); 
        chassis.arcadeDrive(xSpeed, zRotation, true); 
        break; 
      case curvatureDrive:
        // TO DO: Decide if and when we want to use isQuickTurn, and what button to put it on
        zRotation = toDegrees(atan(controller.getY(Hand.kRight)/controller.getX(Hand.kRight))) / 180; 
        xSpeed = controller.getY(Hand.kLeft); 
        isQuickTurn = controller.getBumper(Hand.kRight);
        chassis.curvatureDrive(xSpeed, zRotation, isQuickTurn); 
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
