package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public final class GlobalConfig {
    // Unit conversions
    public final static double MM_PER_INCH = 25.4, CM_PER_INCH = 2.54;

    // Bot and Hardware Measurements
    public final static double TICKS_PER_MOTOR_REVOLUTION = 537.6, MECANUM_CIRCUMFERENCE_MM = 100 * Math.PI;
    public final static double MECANUM_CIRCUMFERENCE_IN = MECANUM_CIRCUMFERENCE_MM / MM_PER_INCH;
    public final static double TICKS_PER_INCH = TICKS_PER_MOTOR_REVOLUTION / MECANUM_CIRCUMFERENCE_IN;
    public final static double TILE_LENGTH_IN = 24, TILE_LENGTH_MM = TILE_LENGTH_IN * MM_PER_INCH;
    // Width: distance between outsides of the Mecanum wheels
    // Length: distance between front of intake and back U channel
    public final static double ROBOT_WIDTH_MM = 427.766, ROBOT_LENGTH_MM = 432;

    // Servo Positions
    public final static double CARTRIDGE_INTAKE_POSITION = 0.45, CARTRIDGE_SHOOTER_POSITION = 0.20, CARTRIDGE_LEVEL_POSITION = 0.30;
    public final static double CARTRIDGE_ARM_NEUTRAL_POSITION = 0.31, CARTRIDGE_ARM_PUSH_RING_POSITION = 0;

    // Servo positions for wobble goal mechanism
    public final static double WOBBLE_GOAL_ARM_TUCKED_POSITION = 1.0, WOBBLE_GOAL_ARM_DOWN_POSITION = 0.39, WOBBLE_GOAL_ARM_OVER_WALL_POSITION = 0.88, WOBBLE_GOAL_MOVING_POSITION = 0.5;
    public final static double WOBBLE_GOAL_CLAW_GRAB_POSITION = 0.16, WOBBLE_GOAL_CLAW_RELEASE_POSITION = 0.38, WOBBLE_GOAL_CLAW_OPEN_WIDE = 0.50;

    // Remember to run the tuner every time!
    public static PIDFCoefficients SHOOTER_PIDF_COEFFICIENTS = new PIDFCoefficients(10, 0, 0, 14.7);

    // Max powers for intake and shooter
    public static final double INTAKE_MAX_POWER = 0.85, SHOOTER_MAX_POWER = 0.88, SHOOTER_LATER_RINGS_POWER = 0.85, SHOOTER_AUTO_POWER = 0.76;
    public final static int SHOOTER_RPM = 4100;

    // Start Position
    public static final Pose2d STARTING_POSITION = new Pose2d(-72 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -51 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(180));

    // Ring Shooting Position
    public static final Pose2d RING_SHOOTING_POSITION = new Pose2d(-12 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -75 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, -0.0139);

    // Delivery Positions
    public static final Pose2d DELIVERY_POINT_A = new Pose2d(6 + (-1) * (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -67 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(135));
    public static final Pose2d DELIVERY_POINT_B = new Pose2d(26 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -32, DELIVERY_POINT_A.getHeading());
    public static final Pose2d DELIVERY_POINT_C = new Pose2d(52 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, DELIVERY_POINT_A.getY() - 3, DELIVERY_POINT_A.getHeading());

    // Pickup position for other wobble goal
    public static final Pose2d COLLECT_OTHER_WOBBLE = new Pose2d(-47 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -25.5 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(0));
    public static final Pose2d COLLECT_OTHER_WOBBLE_FOUR_RINGS = new Pose2d(-42 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -30.5 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(0));

    // How far the robot strafes when attempting to collect the other wobble
    public static final double DISTANCE_STRAFED_TO_WOBBLE = 0.9;

    public static final Pose2d PARKING_POSITION = new Pose2d(9, -36, 0);
}