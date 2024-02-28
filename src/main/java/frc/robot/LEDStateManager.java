package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;


public class LEDStateManager {

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
    private static final int DEFAULT_COLOR = 0;
    private static final int RED_COLOR = 1;
    private static final int GREEN_COLOR = 2;
    private static final int BLUE_COLOR = 3;
    private static final int YELLOW_COLOR = 4;
    private static final int ORANGE_COLOR = 5;

    // Setup LED PWM Outputs
    public static final double ledRed = 0.61;
    public static final double ledOrange = 0.65;
    public static final double ledYellow = 0.69;
    public static final double ledGreen = 0.77;
    public static final double ledBlue = 0.87;
    public static final double ledBlack = 0.99;
    public static final double ledPattern = 0.49;

    private PWMSparkMax blinkinLED = new PWMSparkMax(Constants.blinkinPWMChannel);

    private int currentColor = DEFAULT_COLOR;
    private int nonOverrideState = DEFAULT_COLOR; // Variable to store the current non-override state

    private Timer overrideTimer = new Timer(); // Timer for handling override states

    // Method to set LED color based on the current state
    public void setLEDColor() {
        switch (currentColor) {
            case RED_COLOR:
                blinkinLED.set(ledRed);
                break;
            case GREEN_COLOR:
                blinkinLED.set(ledGreen);
                break;
            case BLUE_COLOR:
                blinkinLED.set(ledBlue);
                break;
            case YELLOW_COLOR:
                blinkinLED.set(ledYellow);
                break;
            case ORANGE_COLOR:
                blinkinLED.set(ledOrange);
                break;
            default:
                blinkinLED.set(ledPattern);
                break;
        }
    }

    // Method to handle state transitions and overrides
    public void handleState(String newState) {
        switch (newState) {
            case "No Game Piece Loaded":
                nonOverrideState = RED_COLOR;
                break;
            case "Game Piece Loaded":
                nonOverrideState = GREEN_COLOR;
                break;
            case "Attempting Generic Operation":
                overrideState(BLUE_COLOR);
                break;
            case "Attempting to pick up game piece":
                overrideState(YELLOW_COLOR);
                break;
            case "Attempting to Shoot Game Piece":
                overrideState(ORANGE_COLOR);
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
        currentColor = DEFAULT_COLOR; // Reset LED state to default color
        overrideTimer.stop(); // Stop the override timer
    }
}