package org.firstinspires.ftc.teamcode;

public final class GlobalConfig {
    // Unit conversions
    public final static double MM_PER_INCH = 25.4, CM_PER_INCH = 2.54;

    // Bot and Hardware Measurements
    public final static double TICKS_PER_MOTOR_REVOLUTION = 537.6, MECANUM_CIRCUMFERENCE_MM = 100 * Math.PI;
    public final static double MECANUM_CIRCUMFERENCE_IN = MECANUM_CIRCUMFERENCE_MM / MM_PER_INCH;
    public final static double TILE_LENGTH_IN = 24, TILE_LENGTH_MM = TILE_LENGTH_IN * MM_PER_INCH;

    // Servo Angles (SimpleServo by FTCLib manages the conversions from angles to positions for us)
    // TODO: Determine these ASAP
    public final static double CARTRIDGE_INTAKE_ANGLE = 0, CARTRIDGE_SHOOTER_ANGLE = 0, CARTRIDGE_LEVEL_ANGLE = 0;
    public final static double CARTRIDGE_ARM_NEUTRAL_ANGLE = 0, CARTRIDGE_ARM_PUSH_RING_ANGLE = 0;
}
