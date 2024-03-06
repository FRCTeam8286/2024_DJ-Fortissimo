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
     *   - Intake Arm Motor: CAN ID 30
     *   - Intake Roller Motor: CAN ID 31
     *   - Left Climber Motor: CAN ID 40
     *   - Right Climber Motor: CAN ID 41
     */

    // CAN ID values for devices attached to CAN bus
    public static final int leftFrontMotorID = 10;
    public static final int leftBackMotorID = 11;
    public static final int rightFrontMotorID = 12;
    public static final int rightBackMotorID = 13;
    public static final int leftShooterRoller = 20;
    public static final int rightShooterRoller = 21;
    public static final int intakeArmID = 30;
    public static final int intakeRollerID = 31;
    public static final int leftClimberMotorID = 40;
    public static final int rightClimberMotorID = 41;

    // LED
    public static final int blinkinPWMChannel = 0;

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

    // Intake Positions
    public static final double intakePosition = 0;
    public static final double ampPosition = 0.25;
    public static final double speakerPosition = 0.50;

    // Define the position tolerance for the arm movement
    public static final double intakePositionTolerance = 0.05;
}