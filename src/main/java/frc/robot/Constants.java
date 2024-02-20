package frc.robot;

public class Constants {
    /**
     * The Constants class provides a centralized storage for various configuration
     * values used throughout the robot's code.
     */

    // Setup Debug Mode
    public static final boolean debug  = true;

    /**
     * CAN Bus Mapping:
    * - Motor Controllers:
    *   - Front Left Motor: CAN ID 10
    *   - Back Left Motor: CAN ID 11
    *   - Front Right Motor: CAN ID 12
    *   - Back Right Motor: CAN ID 13
    *   - Top Intake Roller Motor: CAN ID 20
    *   - Bottom Intake Roller Motor: CAN ID 21
    *   - Top Shooter Roller Motor: CAN ID 30
    *   - Bottom Shooter Roller Motor: CAN ID 31
    */

    // CAN ID values for devices attached to CAN bus
    public static final int leftFrontMotorID = 10;
    public static final int leftBackMotorID = 11;
    public static final int rightFrontMotorID = 12;
    public static final int rightBackMotorID = 13;
    public static final int topIntakeMotorID = 20;
    public static final int bottomIntakeMotorID = 21;
    public static final int topShooterMotorID = 30;
    public static final int bottomShooterMotorID = 31;

    // LED
    public static final int blinkinPWMChannel = 0;

    // Setup some standard LED colors to use with the Blinkin
    /**
     * - LED Color Map:
    *
    *  - ledRed: Indicates the robot is in Teleop mode but doesn't have the game piece loaded
    *  - ledBlue: Indicates the robot is busy initializing or calibrating
    *  - ledBlack: Indicates the robot is stopped or no specific condition
    *  - ledGreen: Indicates the robot is in Teleop mode and has the game piece loaded
    *  - ledPattern: Used when robot is first turned on to show Farwell School Colors
    */
    public static final double ledRed = 0.61;
    public static final double ledGreen = 0.77;
    public static final double ledBlue = 0.87;
    public static final double ledBlack = 0.99;
    public static final double ledPattern = 0.51;

    // Movement Modifiers
    public static final double yModifier = 0.7;
    public static final double xModifier = 0.7;
    public static final double zModifier = 0.5;

    // Deadzones
    public static final double yDeadZone = 0.2;
    public static final double xDeadZone = 0.2;
    public static final double zDeadZone = 0.2;

    // Movement Speed Variable
    public static final double defaultMovementSpeed = 0.5;

    // Shooter Speed Variable
    public static final double shooterSpeed = 0.25;

    // Intake Speed Variable
    public static final double intakeSpeed = 0.25;
}