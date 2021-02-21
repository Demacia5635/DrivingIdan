/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int rightFront = 2;
    public static int rightBack = 1;
    public static int leftFront = 3;
    public static int leftBack = 4;

    public static double wheelDiameter = 0.1524;
    public static double pulsePerRotation = 800;

    public static double pulsesPerMeter = pulsePerRotation / (wheelDiameter * Math.PI);
    public static int gyroPort = 0;

    public static int xboxPort = 0;

    public static double robotLength = 0.575;
    public static double maxVelocity = 4;
    public static double maxRadialAccelaration = 16;
    public static double maxAngularVelocity = Math.PI;

    public static final double kp = 0.01; 
    public static final double ki = 0;
    public static final double ks = 0.797; 
    public static final double kv = 2.54; 
    public static final double ka = 0.862; 
    public static final double kd = 0; 

}
