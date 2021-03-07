package org.firstinspires.ftc.teamcode;

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
    public final static double CARTRIDGE_INTAKE_POSITION = 0.32, CARTRIDGE_SHOOTER_POSITION = 0.11, CARTRIDGE_LEVEL_POSITION = 0.22;
    public final static double CARTRIDGE_ARM_NEUTRAL_POSITION = 0.31, CARTRIDGE_ARM_PUSH_RING_POSITION = 0;

    // Servo positions for wobble goal mechanism
    public final static double WOBBLE_GOAL_ARM_UP_POSITION = 0.97, WOBBLE_GOAL_ARM_DOWN_POSITION = 0.28, WOBBLE_GOAL_ARM_NEUTRAL_POSITION = 0.79;
    public final static double WOBBLE_GOAL_CLAW_GRAB_POSITION = 0.16, WOBBLE_GOAL_CLAW_RELEASE_POSITION = 0.33;

    // Max powers for intake and shooter
    public static final double INTAKE_MAX_POWER = 0.70, SHOOTER_MAX_POWER = 1;
}