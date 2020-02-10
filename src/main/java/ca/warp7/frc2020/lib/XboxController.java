package ca.warp7.frc2020.lib;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;

import static ca.warp7.frc2020.lib.XboxController.ButtonState.None;

/**
 * Handle input from Xbox 360 or Xbox One controllers connected to the Driver Station.
 *
 * This class handles Xbox input that comes from the Driver Station. There is a single
 * class instance for each controller.
 *
 * The state of the controller is updated through the collectControllerData method
 */
public class XboxController {

    public ButtonState aButton          = None;
    public ButtonState bButton          = None;
    public ButtonState xButton          = None;
    public ButtonState yButton          = None;

    public ButtonState leftBumper       = None;
    public ButtonState rightBumper      = None;

    public ButtonState leftStickButton  = None;
    public ButtonState rightStickButton = None;

    public ButtonState backButton       = None;
    public ButtonState startButton      = None;

    public double leftTrigger           = 0.0;
    public double rightTrigger          = 0.0;

    public double leftX                 = 0.0;
    public double leftY                 = 0.0;

    public double rightX                = 0.0;
    public double rightY                = 0.0;

    public DPad dPad                    = new DPad();

    private DriverStation driverStation = DriverStation.getInstance();
    private boolean unplugReported      = false;
    private int port;

    /**
     * Create a new controller helper
     *
     * @param port the port on the Driver Station
     */
    public XboxController(int port) {
        this.port = port;

        HAL.report(FRCNetComm.tResourceType.kResourceType_XboxController, port + 1);
    }

    /**
     * Update the controller data. This should be done once in each periodic function
     * of the robot. This method reports a warning to the Driver Station whenever
     * the controller gets unplugged
     */
    public void collectControllerData() {

        int buttonCount = driverStation.getStickButtonCount(port);
        int axisCount = driverStation.getStickAxisCount(port);

        if (buttonCount >= 10 && axisCount > 5) {
            int buttons      = driverStation    .getStickButtons(port);

            aButton          = aButton          .update((buttons & 1     ) != 0);
            bButton          = bButton          .update((buttons & 1 << 1) != 0);
            xButton          = xButton          .update((buttons & 1 << 2) != 0);
            yButton          = yButton          .update((buttons & 1 << 3) != 0);

            leftBumper       = leftBumper       .update((buttons & 1 << 4) != 0);
            rightBumper      = rightBumper      .update((buttons & 1 << 5) != 0);

            backButton       = backButton       .update((buttons & 1 << 6) != 0);
            startButton      = startButton      .update((buttons & 1 << 7) != 0);

            leftStickButton  = leftStickButton  .update((buttons & 1 << 8) != 0);
            rightStickButton = rightStickButton .update((buttons & 1 << 9) != 0);

            leftX            = driverStation    .getStickAxis(port, 0);
            leftY            = driverStation    .getStickAxis(port, 1);

            leftTrigger      = driverStation    .getStickAxis(port, 2);
            rightTrigger     = driverStation    .getStickAxis(port, 3);

            rightX           = driverStation    .getStickAxis(port, 4);
            rightY           = driverStation    .getStickAxis(port, 5);

            dPad.value       = driverStation    .getStickPOV(port, 0);

            unplugReported   = false;
        } else {
            if (!unplugReported) {
                unplugReported = true;
                DriverStation.reportWarning("The controller on port " + port +
                        "is not plugged in", false);
            }
        }
    }

    /**
     * A ButtonState represents one of the four states of the button
     */
    public enum ButtonState {

        Pressed,  // The button has just changed from up to down

        Released, // The button has just changed from down to up

        HeldDown, // The button was already down and stays down

        None;     // The button was already up and stays up

        /**
         * @return if the button is pressed
         */
        public boolean isPressed() {
            return this == Pressed;
        }

        /**
         * @return if the button is released
         */
        public boolean isReleased() {
            return this == Released;
        }

        /**
         * @return if the button is held down
         */
        public boolean isHeldDown() {
            return this == HeldDown;
        }

        /**
         * @return if the button is kept up
         */
        public boolean isKeptUp() {
            return this == None;
        }

        /**
         * @return if the button is up
         */
        public boolean isUp() {
            return isReleased() || isKeptUp();
        }

        /**
         * @return if the button is down
         */
        public boolean isDown() {
            return isPressed() || isHeldDown();
        }

        /**
         * Update a ButtonState based on an old state and a new value
         *
         * @param newState the new button value
         * @return the appropriate new ButtonState given the inputs
         */
        public ButtonState update(boolean newState) {
            return newState ? isDown() ? HeldDown : Pressed : isUp() ? None : Released;
        }
    }

    /**
     * Represents the directional-pad on the controller
     */
    @SuppressWarnings("unused")
    public static class DPad {
        private int value;

        public int getValue() {
            return value;
        }


        public double getY() {
            return Math.cos(Math.toRadians(value));
        }

        public double getX() {
            return -Math.sin(Math.toRadians(value));
        }
    }
}
