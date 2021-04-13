package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
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
    public final static double CARTRIDGE_INTAKE_POSITION = 0.47, CARTRIDGE_SHOOTER_POSITION = 0.22, CARTRIDGE_LEVEL_POSITION = 0.34;
    public final static double CARTRIDGE_ARM_NEUTRAL_POSITION = 0.28, CARTRIDGE_ARM_PUSH_RING_POSITION = 0;

    // Servo positions for wobble goal mechanism
    public final static double WOBBLE_GOAL_ARM_TUCKED_POSITION = 0.95, WOBBLE_GOAL_ARM_DOWN_POSITION = 0.28, WOBBLE_GOAL_ARM_OVER_WALL_POSITION = 0.8, WOBBLE_GOAL_MOVING_POSITION = 0.4;
    public final static double WOBBLE_GOAL_CLAW_GRAB_POSITION = 0.50, WOBBLE_GOAL_CLAW_RELEASE_POSITION = 0.63, WOBBLE_GOAL_CLAW_OPEN_WIDE = 0.88;

    // Remember to run the tuner every time!
    public static PIDFCoefficients SHOOTER_PIDF_COEFFICIENTS = new PIDFCoefficients(10, 0, 0, 14.51037593985026);

    // Max powers for intake and shooter
    public static final double INTAKE_MAX_POWER = 0.85, SHOOTER_MAX_POWER = 0.88;
    public static int AUTO_SHOOTER_RPM = 3650, TELEOP_SHOOTER_RPM = /*4000*/ 4100;

    // Start Position
    public static final Pose2d STARTING_POSITION = new Pose2d(-72 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -51 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(180));

    // Ring Shooting Position
    public static final Pose2d RING_SHOOTING_POSITION = new Pose2d(-5 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -70.5 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, -Math.toRadians(3.5));

    // Delivery Positions
    public static final Pose2d DELIVERY_POINT_A = new Pose2d(12 + (-1) * (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -69 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(135));
    public static final Pose2d DELIVERY_POINT_B = new Pose2d(32 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -34, DELIVERY_POINT_A.getHeading());
    public static final Pose2d DELIVERY_POINT_C = new Pose2d(60 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, DELIVERY_POINT_A.getY() - 3.5, DELIVERY_POINT_A.getHeading());

    // Pickup position for other wobble goal
    public static final Pose2d COLLECT_OTHER_WOBBLE = new Pose2d(-45 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -29.5 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(0));
    public static final Pose2d COLLECT_OTHER_WOBBLE_FOUR_RINGS = new Pose2d(-42 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -29.5 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(5));

    // How far the robot strafes when attempting to collect the other wobble
    public static final double DISTANCE_STRAFED_TO_WOBBLE = 2;

    public static final Pose2d PARKING_POSITION = new Pose2d(11, -36, 0);
}