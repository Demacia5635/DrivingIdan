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
    public static int rightFront = 1;
    public static int rightBack = 2;
    public static int leftFront = 3;
    public static int leftBack = 4;
    public static double pulsesPerMeter = 0.5;
    public static int gyroPort = 0;

    public static int xboxPort = 0;

    public static double robotLength = 0.575;
    public static double maxVelocity = 4;
    public static double maxRadialAccelaration = 1;
    public static double maxAngularVelocity = 4*Math.PI/180;
}
