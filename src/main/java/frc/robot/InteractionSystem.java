package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;

public class InteractionSystem {
    /**
     * Controls interaction mechanisms like intakes and shooters, offering methods
     * for operation and management of these systems within the robot.
     */
    private CANSparkMax leftShooterRoller, rightShooterRoller, intakeRoller, intakeArm;
    private boolean isIntakeRunning = false;
    private double intakeStartTime = 0;
    private double intakeDuration = 0;
    private boolean isShooterRunning = false;
    private double shooterStartTime = 0;
    private double shooterDuration = 0;

    public InteractionSystem() {
        // Initializes motors for intake and shooter systems as brushless motors
        leftShooterRoller = new CANSparkMax(Constants.leftShooterRoller, MotorType.kBrushless);
        rightShooterRoller = new CANSparkMax(Constants.rightShooterRoller, MotorType.kBrushless);
        intakeRoller = new CANSparkMax(Constants.intakeRollerID, MotorType.kBrushless);
        intakeArm = new CANSparkMax(Constants.intakeArmID, MotorType.kBrushless);

        // Configure initial motor settings (e.g., inversion)
        configureMotors();
    }

    private void configureMotors() {
         // Sets the direction of motor rotation to match physical setup
        leftShooterRoller.setInverted(true); 
        rightShooterRoller.setInverted(false);
        intakeRoller.setInverted(true);
        intakeArm.setInverted(false);
    }

    public void moveArmForward(double speed) {
        // Sets intake motor speeds; positive values intake, negative values expel
        intakeArm.set(speed);
    }

    public void moveArmBackward(double speed) {
        // Sets intake motor speeds; positive values intake, negative values expel
        intakeArm.set(-speed);
    }

    public void runIntake(double speed) {
        // Sets intake motor speeds; positive values intake, negative values expel
        intakeRoller.set(speed);
    }

    public void stopIntake() {
        // Stops the intake motors
        intakeRoller.set(0);
    }
    public void timedIntake(double speed, double duration) {
        // Starts intake motors and schedules it to stop after a duration
        this.runIntake(speed); // Start the intake
        isIntakeRunning = true; // Flag to track intake state
        intakeStartTime = Timer.getFPGATimestamp(); // Record start time for duration tracking
        intakeDuration = duration; // Set how long the intake should run
    }

    public void runShooter(double speed) {
         // Sets shooter motor speeds; positive for shooting, negative could reverse feed
        leftShooterRoller.set(speed);
        rightShooterRoller.set(speed);
    }

    public void stopShooter() {
        // Stops the shooter motors
        leftShooterRoller.set(0);
        rightShooterRoller.set(0);
    }
    public void timedShooter(double speed, double duration) {
        // Starts shooter motors and schedules it to stop after a set duration
        this.runShooter(speed); // Start the intake
        isShooterRunning = true;
        shooterStartTime = Timer.getFPGATimestamp(); // Current time
        shooterDuration = duration;
    }

    // Call this method from the Robot class's periodic methods to update intake and shooter states
    public void update() {
        // Periodically called to check and update the state of intake and shooter based on timers
        double currentTime = Timer.getFPGATimestamp();
        // Checks if intake has exceeded its run duration, then stops it
        if (isIntakeRunning && ((currentTime - intakeStartTime) >= intakeDuration)) {
            this.stopIntake();
            isIntakeRunning = false; // Reset intake running flag
        }
        // Checks if shooter has exceeded its run duration, then stops it
        if (isShooterRunning && ((currentTime - shooterStartTime) >= shooterDuration)) {
            this.stopShooter();
            isShooterRunning = false; // Reset shooter running flag
        }
    }
}
