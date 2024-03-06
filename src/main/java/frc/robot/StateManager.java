package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;


public class StateManager {

    /**The robot can be in a few states, with corresponding LED colors
     * 
     * These are default states that may get overrided:
     * 0 - Default Mode = Flashing School Colors
     * 1 - No Game Piece Loaded = Solid Red
     * 2 - Game Piece Loaded = Solid Green
     * 
     * These states will override the default states, and may persist:
     * 3 - Attempting Generic Operation = Solid Blue
     * 4 - Attempting to pick up game piece = Solid Yellow
     * 5 - Attempting to Shoot Game Peice = Solid Orange
    */

    // Setup LED State Codes
    private static final int defaultColor = 0;
    private static final int redColor = 1;
    private static final int greenColor = 2;
    private static final int blueColor = 3;
    private static final int yellowColor = 4;
    private static final int orangeColor = 5;

    // Setup LED PWM Outputs
    public static final double ledRed = 0.61;
    public static final double ledOrange = 0.65;
    public static final double ledYellow = 0.69;
    public static final double ledGreen = 0.77;
    public static final double ledBlue = 0.87;
    public static final double ledBlack = 0.99;
    public static final double ledPattern = 0.49;

    private PWMSparkMax blinkinLED = new PWMSparkMax(Constants.blinkinPWMChannel);

    private int currentColor = defaultColor;
    private int nonOverrideState = defaultColor; // Variable to store the current non-override state

    private Timer overrideTimer = new Timer(); // Timer for handling override states

    private int currentState;
    private int currentNonOverrideState;

    // Method to set LED color based on the current state
    public void setLEDColor() {
        switch (currentColor) {
            case redColor:
                blinkinLED.set(ledRed);
                break;
            case greenColor:
                blinkinLED.set(ledGreen);
                break;
            case blueColor:
                blinkinLED.set(ledBlue);
                break;
            case yellowColor:
                blinkinLED.set(ledYellow);
                break;
            case orangeColor:
                blinkinLED.set(ledOrange);
                break;
            default:
                blinkinLED.set(ledPattern);
                break;
        }
    }

    // Method to handle state transitions and overrides
    public void setState(int desiredState) {
        currentState = desiredState;
        switch (desiredState) {
            case 1:
                currentNonOverrideState = desiredState;
                nonOverrideState = redColor;
                break;
            case 2:
                currentNonOverrideState = desiredState;
                nonOverrideState = greenColor;
                break;
            case 3:
                overrideState(blueColor);
                break;
            case 4:
                overrideState(yellowColor);
                break;
            case 5:
                overrideState(orangeColor);
                break;
            default:
                // Handle other states or default behavior
                break;
        }
    }

    private void overrideState(int color) {
        // Override current state with a specific color for a duration
        currentColor = color;
        overrideTimer.reset();
        overrideTimer.start();
    }

    public void update() {
        // Update LED state based on timers and non-override state
        if (overrideTimer.hasElapsed(1.0)) {
            // If override timer has elapsed, revert to non-override state
            currentColor = nonOverrideState;
        }
        setLEDColor(); // Update LED color
    }

    public void clearOverrideState() {
        currentState = currentNonOverrideState;
        currentColor = defaultColor; // Reset LED state to default color
        overrideTimer.stop(); // Stop the override timer
    }
}