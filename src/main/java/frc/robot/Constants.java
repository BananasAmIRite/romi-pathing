// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double robotPeriod = 0.02 * 5; 

    public static class RomiDrivetrain {

        public static final double scaleDownMultiplier = 1/4.0; // scale the robot down 

        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterMeters = 70.0/1000 / scaleDownMultiplier; // 70 mm

        public static final double distancePerPulse = (Math.PI * kWheelDiameterMeters) / kCountsPerRevolution; 


        // tuned constants (use https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization-sysid)
        public static final double kTrackWidth = 0.141; 
        public static final PIDConstants kWheelPIDLeft = new PIDConstants(0/*0.38417*/, 0); 
        public static final SimpleMotorFeedforward driveFFLeft = new SimpleMotorFeedforward(0, 5.0362, 0 /*0.35412*/); 

        public static final PIDConstants kWheelPIDRight = new PIDConstants(0/*0.38417*/, 0); 
        public static final SimpleMotorFeedforward driveFFRight = new SimpleMotorFeedforward(0, 5.0362, 0/*0.35819*/); 

        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth); 

    }

    public static class Trajectory {
        public static final PIDConstants kDrivePID = new PIDConstants(3, 0, 0);
        public static final PIDConstants kTurnPID = new PIDConstants(3, 0, 0);  
    }

    public static class XRPDrivetrain {
        
        public static final double scaleDownMultiplier = 1/4.0; // scale the robot down 


        public static final double kCountsPerMotorShaftRev = 12.0;
        private static final double kGearRatio =
        (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
        public static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
        public static final double kWheelDiameterMeters = 60.0/1000 / scaleDownMultiplier; // 70 mm

        public static final double distancePerPulse = (Math.PI * kWheelDiameterMeters) / kCountsPerRevolution; 


        // tuned constants (use https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization-sysid)
        public static final double kTrackWidth = 0.155 / scaleDownMultiplier; 
        public static final PIDConstants kWheelPIDLeft = new PIDConstants(0/*2.278 */, 0); 
        public static final SimpleMotorFeedforward driveFFLeft = new SimpleMotorFeedforward(1.6542, 3.8371, 0.99265); 

        public static final PIDConstants kWheelPIDRight = new PIDConstants(0/*2.278 */, 0); 
        public static final SimpleMotorFeedforward driveFFRight = new SimpleMotorFeedforward(1.6957, 4.0648, 0.89423); 

        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth); 
    }
}
