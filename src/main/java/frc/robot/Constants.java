// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;
import swervelib.math.Matter;

import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MASS = (90) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class AutonConstants {

        public static final com.pathplanner.lib.util.PIDConstants TRANSLATION_PID = new com.pathplanner.lib.util.PIDConstants(0.7, 0, 0);
        public static final com.pathplanner.lib.util.PIDConstants ANGLE_PID = new com.pathplanner.lib.util.PIDConstants(0.9, 0, 0.01);
        public static final PIDConstants ANGLE_PID_L = new PIDConstants(0.4, 0, 0.01, 0, 0.01);
        public static final List<Double> FOUR_NOTE_ANGLES = List.of(0.12, 0.13, 0.12);
        public static final List<Double> FOUR_NOTE_SPEEDS_TOP = List.of(8000., 8000., 8000.);
        public static final List<Double> FOUR_NOTE_SPEEDS_BOTTOM = List.of(7300., 8100., 7300.);

        public static final List<Double> THREE_NOTE_SPEEDS_TOP = List.of(10000.,10000.);
        public static final List<Double> THREE_NOTE_SPEEDS_BOTTOM = List.of(8000.,10000.);
        public static final List<Double> THREE_NOTE_ANGLES = List.of(0.11, 0.11);

    }
//
    public static final class DrivebaseConstants {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static final class LimelightConstants {
        public static final double LIMELIGHT_HEIGHT = 0.44; // meters
        public static final double LIMELIGHT_ANGLE = 30; // degrees
        public static final double TARGET_HEIGHT = 1.37; // meters

        public static final double SPEAKER_HEIGHT = 1.98; //meters

        public static final double TARGET_HEIGHT_DIFFERENCE = TARGET_HEIGHT - LIMELIGHT_HEIGHT;
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;

        public static final double FEEDER_DELAY = 0.8;
    }

    public static class ShooterConstants{
        public static final int SHOOTER_ARM = 16;
        public static final int SHOOTER_TOP = 18;
        public static final int SHOOTER_BOTTOM = 17;
        public static final int SHOOTER_FEED = 19;

        public static final boolean ARM_INVERTED = true;
        public static final boolean TOP_INVERTED = true;
        public static final boolean BOTTOM_INVERTED = true;
        public static final boolean FEEDER_INVERTED = false;

        public static final PIDConstants ARM_PID = new PIDConstants(3.0, 4.0, 0.0, 0.03, 0.01);
        public static final FFConstants ARM_FF = new FFConstants(0.0, 0.04, 0.0, 0.0);

        public static final PIDConstants TOP_PID = new PIDConstants(0.005, 0.0, 0.0);
        public static final FFConstants TOP_FF = new FFConstants(0.13, 0.001046622264);
        public static final PIDConstants BOTTOM_PID = new PIDConstants(0.005, 0.0, 0.0);
        public static final FFConstants BOTTOM_FF = new FFConstants(0.13, 0.00107003891);

        public static final double SHOOTER_RPM = 8000;
        public static final double AMP_RPM = 5000;
        public static final double AUTO_RPM_TOP = 8000;
        public static final double AUTO_RPM_BOT = 8000;
        public static final double AUTO_RPM = 10000;
        public static final double FEEDER_SPEED = 0.6;
        public static final double FEEDER_SPEED_SLOW = 0.2;

        public static final double ARM_ABSOLUTE_OFFSET = -0.88;

        public static final double DEFAULT_SPEAKER_SHOT = 0.075;
        public static final double AMP_POS = 0.3;
        public static final double CLIMB_POS = 0.3;
        public static final double FEED_POS = 0.115;
        public static final double STOW_POS = -0.00;
        public static final double SAVE_POS = -0.07;
    }

    public static class IntakeConstants{
        // ids
        public static final int INTAKE_ARM = 14;
        public static final int INTAKE_ROLLER = 15;

        public static final boolean ARM_INVERTED = false;
        public static final boolean ROLLER_INVERTED = false;

        public static final PIDConstants ARM_PID = new PIDConstants(2.5, 1.0, 0.1, 0.05, 0.01);
        public static final FFConstants ARM_FF = new FFConstants(0.0, 0.05, 0.0, 0.0);

        public static final double ROLLER_INTAKE_SPEED = 0.45;
        public static final double ROLLER_INTAKE_HOLD = 0.2;
        public static final double ROLLER_OUTTAKE_SPEED = -0.2;
        public static final double ARM_GROUND_POSITION = -0.07;
        // public static final double ARM_INTAKE_POSITION = 0.5;
        public static final double ARM_STOW_POSITION = 0.16;
        public static final double ARM_TRANSFER_POSITION = 0.3;
        public static final double ARM_TOLERANCE = 0.05;
        public static final double ARM_ABSOLUTE_OFFSET = -0.43;
    }

    public static class ClimberConstants {
        public static final int WINCH = 20;
        public static final boolean WINCH_INVERTED = false;
    }
}
/*
⠀⠀⠀⠀⠀⠀⠀⠀⢠⣾⣯⣟⡆⢸⢠⣯⡃⠀⠀⠀⠀⢹⣆⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⢈⣿⠉⢹⢉⢹⡌⢉⡜⠚⠭⠤⠋⠑⣼⣿⢟⡿⣿⣻⣯⣸⡇⣿⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⣦⡄⠀⠀⠀⠀⢸⡿⢄⣈⠀⢸⣿⣿⠃⠀⠀⠀⠀⡎⣿⡄⠀⠀⠀⠀⠀⢀⣀⣤⣶⣾⣿⠟⠙⢸⠯⣈⠋⠉⠀⣼⣾⣦⣤⣶⣿⣶⡟⢼⣺⣝⠾⠕⠙⠻⣽⣖⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠘⡟⠙⢦⡀⠀⠖⠛⠛⠺⠶⣉⢦⣷⡇⠀⠀⠀⠀⠀⠸⡸⣷⡆⣀⣤⣴⣾⡿⠟⠛⠉⠀⠈⠛⠛⠻⢦⣼⣶⣶⣾⣿⣿⣿⣿⣿⣿⣿⣷⣖⣬⣴⢷⣶⡴⣶⣿⣦⣤⡙⠻⣶⣄⠀⠀⠀⠀⠀⠀⠀
⠀⠈⠀⠀⠀⠹⢦⡀⠀⠀⠀⠀⠀⠙⢿⣿⡄⠀⠀⠀⠀⠀⢣⢻⣿⡿⠖⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠻⢿⣿⣿⣿⣿⣿⣿⣿⡏⣾⣿⣿⡜⢿⣿⣮⢻⣿⣿⣿⣷⣾⢻⡆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠈⠙⢦⡀⠀⠀⠀⠀⠀⠙⢷⡀⠀⠀⢀⣠⣼⣿⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⣴⣶⠿⠟⠛⠋⠉⠁⠀⣼⢰⡿⢿⣿⣿⣮⣛⢿⡇⣿⣿⣿⢿⣷⡿⠁⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡀⠀⠀⠀⠀⠀⣳⣰⣾⣿⣿⣿⣿⠃⠀⠀⠀⠀⠀⠀⠀⣀⣤⣴⣶⣿⣿⣿⣍⣄⣀⣀⣀⡐⣀⣠⣠⣄⣿⢸⣿⣿⣿⣿⣿⣯⡟⣰⣷⣿⣿⣿⠟⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢦⡀⢠⣴⣿⣿⣿⣿⣿⣿⣿⡏⠀⠀⠀⢀⣠⣴⣾⣿⣿⣿⣿⣿⠿⠿⠿⠿⢿⣿⡛⢟⠛⠛⠛⠛⠛⠿⣿⡹⣻⠷⢿⣛⣭⣾⠟⢿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⣠⣴⣾⣿⣿⣿⣿⡿⠟⠋⠁⠀⠀⠀⠀⠀⠀⠉⡻⣮⡳⣄⠀⠀⠀⠀⠨⡻⡻⡟⢻⣿⡿⣇⠀⠈⢿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠁⠀⠀⠀⠀⠀⠀⠀⢀⣠⣾⠟⠙⠁⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⠋⠁⠀⠀⠀⠀⠀⠀⢀⡠⠀⠤⠤⠐⠌⢿⣾⣦⡀⠀⠀⠀⢳⡙⢟⡄⠹⡿⡇⡆⠀⠈⣿⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀
⣌⠢⣄⠀⠀⠀⠀⣰⣿⢿⠃⠀⠀⠀⠐⢿⣿⣿⣿⣿⠟⠋⣁⣾⣿⡟⠉⠀⠀⠀⠀⠀⢀⡈⠲⢜⡛⠥⠄⠖⠉⠁⢀⣀⣀⣘⢳⣵⡄⠀⠀⠀⢷⠘⣽⡄⢱⡇⢣⠀⠀⠘⣿⣿⣿⣦⡀⠀⠀⠀⠀⠀
⠈⠳⣌⠳⣄⣤⣾⠟⢡⠏⠀⠀⠀⠀⢀⣬⡿⠟⠉⠀⠀⣠⡿⠋⠘⢿⡦⣄⠀⢀⠤⠐⠁⠀⠀⠀⠹⢶⣤⣶⠾⠛⠉⠁⠀⢀⣰⠙⣿⡄⠀⠀⠘⡇⢸⢷⠘⣷⠀⠀⠀⠀⢹⣿⣿⣿⣿⣷⣄⡀⠀⠀
⠀⠀⠈⠳⣼⡽⠋⢀⡞⠀⠀⠀⣠⣴⠟⠉⢀⡄⠀⢀⣴⡋⠀⠀⠀⡄⠙⠮⣷⣤⡤⠜⠁⠀⠀⣠⡾⠟⠉⠛⠿⣿⣭⠿⠿⣿⣿⣿⣽⣷⡀⠀⠀⣽⠸⣛⣆⡇⠀⠀⠀⠀⠀⢿⣿⣧⡙⣿⣿⣿⣶⣤
⠀⠀⠀⠀⡿⠀⠀⣼⠁⢀⣤⡾⠋⠀⠀⠀⠘⣿⣤⠞⠀⠙⢦⣄⢸⠁⠀⠀⠈⠑⠿⣶⣤⣤⣾⠋⠀⠀⣀⣀⣀⠁⠈⠙⢒⣦⣤⣼⣥⣬⢷⡀⠀⢸⠀⣿⣿⡇⠀⠀⠀⠀⠀⠸⣷⠙⢿⣌⠹⠟⠙⠻
⠀⠀⠀⢰⠃⠀⣸⢁⣴⣿⠋⠀⠀⠀⠀⠀⠀⡾⣷⡀⣀⢀⡴⠙⢷⣍⠢⢄⠏⠉⠙⠮⠉⠻⢤⡛⠉⠉⠉⠁⢀⣬⣭⣿⣿⣿⣿⠿⢿⣿⡘⢧⠀⢸⣸⡏⣿⠁⠀⢠⠀⠀⠀⡇⢻⡤⠀⠙⣷⣄⠀⠀
⠀⠀⢀⡏⢀⣼⣵⣿⣿⣿⡀⠀⠀⠀⠀⠀⡼⠁⢈⢿⡆⠑⠦⣀⠀⣙⣻⢄⡀⠀⠀⠀⠀⠀⠀⠉⠳⢦⣀⣞⣵⣿⣿⣫⡯⡭⡿⢷⠀⣿⠃⠸⡇⠘⣿⢧⡏⠀⠀⠏⠀⣀⣀⢧⠸⣧⡀⠀⠀⠙⢷⡔
⠀⠀⣸⢡⣮⣾⣿⣿⣿⡇⣿⡄⠀⠀⠀⣸⠁⣠⠏⡼⣿⣶⣿⣂⣥⡤⡄⠀⠉⠛⢿⣵⣦⣀⣀⣀⡀⣀⣈⣙⣻⢿⣣⡮⡅⠈⠛⢽⣧⡏⠀⠀⣷⣼⡿⣾⡁⠀⠐⠀⠀⠀⠀⢈⣱⡇⠙⢷⣤⡀⠀⠉
⠀⢠⣧⣿⣿⣿⣿⣿⣿⣇⡇⢷⠀⠀⣸⢻⡂⠃⡰⠁⣧⣷⠟⠻⣕⡦⢌⣑⠲⠤⣄⣈⠙⠢⢤⣀⡀⠀⢸⠏⣠⠈⠸⣟⣎⣄⡴⠚⠁⠀⠀⢀⣿⠟⣧⡿⠛⣶⣦⠤⠤⠶⣾⠛⠉⠀⠀⠀⠻⣟⠳⢤
⠀⡜⣾⣿⣿⣿⣿⣿⣿⣿⢹⢻⣇⢠⡏⠀⢳⡐⠃⣰⠏⣷⠀⠀⠀⠙⠳⠮⣏⡲⠒⠭⠭⠍⣒⡒⡻⠿⣟⣋⣁⣀⠤⠀⠋⠁⠀⠀⠀⠀⠤⠞⢁⣴⠟⣁⡼⠋⠈⣇⠀⠀⠙⣆⠀⠀⠀⠀⠀⠙⢧⡀
⠀⠻⣛⠿⣿⣿⣿⣿⣿⣿⡏⡎⢿⣾⣱⡔⣌⢷⣜⠁⠀⡇⡂⠀⠀⣠⣴⣒⣒⣉⣙⠓⠒⠒⠤⠴⠔⠛⠛⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠴⣿⣡⣾⣿⡀⠀⠀⢹⠀⠀⠀⢹⣆⠀⠀⠀⠀⠀⠈⢳
⠀⠀⠈⠉⠐⠚⠫⠽⠿⠿⢧⠸⡜⣾⣇⠹⡜⢮⢳⡵⠄⣇⠡⢤⣾⣿⣿⣿⡿⣿⣯⡉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠤⠴⠟⠋⠁⡿⣿⣧⠀⠀⠸⣿⠐⣰⢻⣿⡆⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⣷⡈⢿⡆⠙⢦⡳⣿⣦⣻⡤⣿⡿⠻⣗⣤⣯⡀⠻⢷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠃⢿⣹⡄⠀⠀⣿⠀⣿⠀⡟⣿⡀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣇⢷⢳⡀⠙⢤⠀⠉⠚⠻⣿⣾⢿⡧⣀⠈⠺⣷⣇⣠⠞⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⠏⠀⢸⡏⢷⠀⠀⣿⡄⢹⠀⢧⠸⣇⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡼⡆⠳⡄⠀⢕⢤⡀⠀⠈⠻⢷⣍⡑⠋⠶⠲⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⠔⡿⠀⠀⠀⠀⠀⣠⡏⠀⠀⠀⣿⣜⣧⢠⡏⠁⠸⠀⠈⠇⣻⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⣿⡀⠙⢦⡀⠙⢿⣶⣤⠀⠀⠉⠻⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡤⠒⠉⢀⡼⠁⠀⠀⠀⠀⣴⠋⠀⣀⡴⡞⠛⣿⡼⣾⡇⠀⠀⠀⠀⢸⠘⡧⡀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣿⣰⡷⠛⢦⡈⠓⢍⠻⣶⢦⣤⣌⣻⣷⣦⣤⡤⠀⠀⠀⠀⠀⠀⠀⠉⠒⠒⠋⠁⠀⠀⠀⠀⢀⣴⠃⣠⡾⠋⠁⠀⠀⢻⣿⣾⠃⠀⡅⠀⠀⣼⡀⣷⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣼⡿⢁⠀⠈⣿⢲⣤⢕⡺⣷⣇⢸⠀⠀⢸⡲⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡞⣡⡾⠉⠀⠀⠀⠀⠀⠈⢻⣿⡆⢰⠁⠀⡴⡏⡇⢹⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣖⡿⠃⠀⠀⠀⢀⣾⠀⠈⡟⠺⣭⣟⣻⢤⠀⠀⡇⣿⠋⠙⢶⣶⡤⠤⠤⠤⣤⣤⣀⣀⣀⣀⡴⠋⣼⠟⠀⠀⠀⠀⠀⠀⠀⣠⣾⡇⡸⡟⠀⣘⡰⡇⠇⢸⡇⠀⠀⠀
⠠⠤⣚⣛⠷⣤⠀⠀⣀⢤⣺⣿⡏⠀⠀⠀⢀⡴⠋⢸⠀⠀⠳⢸⠃⢸⢹⡉⠉⠉⠀⢸⠀⠀⠈⣿⡀⠀⠀⠀⠀⠀⠈⣿⣧⡀⢀⡾⠃⠀⠀⠀⠀⠀⠀⢀⣾⣿⡿⣄⡿⡴⠉⠀⠀⡇⠀⢸⣷⣶⣶⣤
⠄⠀⠉⠉⢉⣉⣩⣭⣷⣿⣿⣿⣅⣀⡤⠚⠁⠀⠀⣼⠀⠀⠀⣼⠀⣼⠐⡇⠀⢀⠀⣾⠀⠀⠀⡇⡇⠀⠀⠀⠀⣠⣿⣿⣿⣷⡞⠁⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⢃⡹⡼⠀⠀⠀⠀⣿⠀⣼⣿⣿⡿⠛
⠃⠀⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⡏⠀⠀⢀⡏⠀⡟⡀⢷⠀⡾⢀⡿⠀⠀⠀⢡⢸⠀⣀⣴⣾⣿⣿⣿⣿⣿⣗⣪⡖⢶⡆⠀⢀⣾⣿⣿⣿⠏⢡⠳⠁⠀⠀⠀⢰⢻⣠⠇⣿⠟⠁⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⠃⠀⢀⡾⠀⢠⣷⡇⠸⡶⡇⣸⠃⠀⠀⠀⣘⣼⣾⣿⣿⣿⣿⡿⣻⣛⣿⣭⡿⢷⠾⠃⣠⣾⣿⠿⢫⡏⢀⡏⠀⠀⠀⠀⢀⡪⣿⠸⢸⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠏⠀⣠⠟⠀⠀⣼⣾⢱⡀⢻⣡⡏⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⠃⠻⠟⣹⣿⣷⠀⢀⣴⡿⣿⠟⠁⣤⠟⢀⡞⠀⠀⠀⠀⠔⠋⢠⡇⠁⣾⠇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠟⣠⠾⠃⠀⠀⢠⡇⣿⣈⢧⠈⢿⣇⣴⣿⣿⣿⣿⣿⣿⢟⣿⠇⠀⢀⣼⣿⣿⣿⣷⠿⣫⠞⠁⣠⣾⠋⢠⡞⠀⠀⠀⠀⠀⠀⠀⣼⡅⢠⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣾⠵⠊⠁⠀⠀⠀⠀⣼⢱⣿⡈⢪⢧⡈⢿⣱⣿⣿⣿⣿⣿⢫⢸⣿⠀⣤⠟⠋⢡⣴⠟⡡⠊⢡⡠⣾⠟⢁⡴⠛⠀⠀⠀⠀⠀⢀⠀⣰⡟⢀⣼⡏⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡟⢸⡿⡇⠀⠈⢳⡈⢿⣿⣿⣿⠋⠀⢸⠈⣿⡟⠂⣠⡾⢋⡴⠊⠀⠚⣻⠞⢁⣴⠟⢀⠔⠁⠀⠀⠀⢀⠌⣴⣿⡡⠾⠋⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡇⠀⡇⢳⠀⠀⠀⠹⢄⢻⡟⠁⠀⠀⢸⠀⢻⣧⡾⢋⣐⣋⣀⣬⣔⣯⠴⢺⡟⠁⠀⠀⠀⠀⠀⢀⡠⠮⢞⡟⠉⠘⠁⠀⠀⠀⠀⠀⠀
*/
