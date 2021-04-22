package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    public final static double CARTRIDGE_INTAKE_POSITION = 0.47, CARTRIDGE_SHOOTER_POSITION = 0.21, CARTRIDGE_LEVEL_POSITION = 0.34;
    public final static double CARTRIDGE_ARM_NEUTRAL_POSITION = 0.28, CARTRIDGE_ARM_PUSH_RING_POSITION = 0;

    // Servo positions for wobble goal mechanism
    public final static double WOBBLE_GOAL_ARM_TUCKED_POSITION = 0.95, WOBBLE_GOAL_ARM_DOWN_POSITION = 0.28, WOBBLE_GOAL_ARM_OVER_WALL_POSITION = 0.8, WOBBLE_GOAL_MOVING_POSITION = 0.4, WOBBLE_GOAL_MOVING_FOUR_RINGS=0.6;
    public final static double WOBBLE_GOAL_CLAW_GRAB_POSITION = 0.50, WOBBLE_GOAL_CLAW_RELEASE_POSITION = 0.63, WOBBLE_GOAL_CLAW_OPEN_WIDE = 0.88;

    // Remember to run the tuner every time!
    public static PIDFCoefficients SHOOTER_PIDF_COEFFICIENTS = new PIDFCoefficients(10, 0, 0, 14.51037593985026);

    // Max powers for intake and shooter
    public static final double INTAKE_MAX_POWER = 0.85, SHOOTER_MAX_POWER = 0.88;
    public static int AUTO_SHOOTER_RPM = 3650, HIGH_GOAL_SHOOTER_RPM = /*4100*/ 4100, POWER_SHOT_SHOOTER_RPM=3450;

    // Start Position
    public static final Pose2d STARTING_POSITION = new Pose2d(-72 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -51 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(180));

    //Intermediate Position
    public static final Vector2d INTERMEDIATE_POSITION=new Vector2d(-48 + GlobalConfig.ROBOT_LENGTH_MM / GlobalConfig.MM_PER_INCH / 2.0, -62 + (GlobalConfig.ROBOT_WIDTH_MM / GlobalConfig.MM_PER_INCH / 2.0));
    // Ring Shooting Position
    public static final Pose2d RING_SHOOTING_POSITION = new Pose2d(8 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -50 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, -Math.toRadians(3.5));

    // Delivery Positions
    public static final Pose2d DELIVERY_POINT_A = new Pose2d(12 + (-1) * (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -69 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(135));
    public static final Pose2d DELIVERY_POINT_B = new Pose2d(39 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -26, DELIVERY_POINT_A.getHeading());
    public static final Pose2d DELIVERY_POINT_C = new Pose2d(66 - (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, DELIVERY_POINT_A.getY() + 12, DELIVERY_POINT_A.getHeading());

    // Pickup position for other wobble goal
    public static final Pose2d COLLECT_OTHER_WOBBLE = new Pose2d(-50 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -44.0 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(330));
    public static final Pose2d COLLECT_OTHER_WOBBLE_ONE_RING = new Pose2d(-47.25 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -27.0 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(5));
            //Proposed Change:new Pose2d(-49 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -25.0 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(25))
    public static final Pose2d COLLECT_OTHER_WOBBLE_FOUR_RINGS = new Pose2d(-48 + (ROBOT_LENGTH_MM / MM_PER_INCH) / 2.0, -23.0 + (ROBOT_WIDTH_MM / MM_PER_INCH) / 2.0, Math.toRadians(25));

    // How far the robot strafes when attempting to collect the other wobble
    //public static final double DISTANCE_STRAFED_TO_WOBBLE = 2;

    public static final Vector2d PARKING_POSITION = new Vector2d(8.0, -36.0);
}