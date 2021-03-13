 package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;

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
    public final static double CARTRIDGE_INTAKE_POSITION = 0.38, CARTRIDGE_SHOOTER_POSITION = 0.15, CARTRIDGE_LEVEL_POSITION = 0.22;
    public final static double CARTRIDGE_ARM_NEUTRAL_POSITION = 0.31, CARTRIDGE_ARM_PUSH_RING_POSITION = 0;

    // Servo positions for wobble goal mechanism
    public final static double WOBBLE_GOAL_ARM_TUCKED_POSITION = 0.97, WOBBLE_GOAL_ARM_DOWN_POSITION = 0.28, WOBBLE_GOAL_ARM_OVER_WALL_POSITION = 0.825, WOBBLE_GOAL_MOVING_POSITION = 0.42;
    public final static double WOBBLE_GOAL_CLAW_GRAB_POSITION = 0.16, WOBBLE_GOAL_CLAW_RELEASE_POSITION = 0.38, WOBBLE_GOAL_CLAW_OPEN_WIDE = 0.50;

    // Max powers for intake and shooter
    public static final double INTAKE_MAX_POWER = 0.80, SHOOTER_MAX_POWER = 0.90, SHOOTER_LATER_RINGS_POWER = 0.875;

    // Start Position
    public static final Pose2d STARTING_POSITION = new Pose2d(-72 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -51 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(180));

    // Ring Shooting Position
    public static final Pose2d RING_SHOOTING_POSITION = new Pose2d(-12 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -72 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, -0.0139);

    // Delivery Positions
    public static final Pose2d DELIVERY_POINT_A = new Pose2d(2 + (-1) * (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -69 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(135));
    public static final Pose2d DELIVERY_POINT_B = new Pose2d(28 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -42, DELIVERY_POINT_A.getHeading());
    public static final Pose2d DELIVERY_POINT_C = new Pose2d(52 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, DELIVERY_POINT_A.getY() - 7, DELIVERY_POINT_A.getHeading());

    // Pickup position for other wobble goal
    public static final Pose2d COLLECT_OTHER_WOBBLE = new Pose2d(-48 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -28.0 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(0));

    // How far the robot strafes when attempting to collect the other wobble
    public static final double DISTANCE_STRAFED_TO_WOBBLE = 3;

    public static final Pose2d PARKING_POSITION = new Pose2d(12, -36, 0);
}