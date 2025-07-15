package frc.robot.lib.leds;

import edu.wpi.first.math.MathUtil;
/**
 * This is the controller for all needs regarding LEDS. Once you create your LED
 * object, you must create sections using {@link LedController#addSection} in
 * your constructor. Finally, include {@link LedController#updateLeds} in your periodic
 */
public class LedController extends LedModes {

    /** The total length (in LEDs) you would like to operate. */
    public static int ledLength;

    /** The PWM port in which the leds are plugged into the RoboRio. */
    public static int ledPort;

    /**
     * The brightness level of your LEDs. LEDs can draw a large amount of current,
     * so it is wise to limit the brightness when controlling a large system like an
     * FRC robot.
     * This is a value between 0 and 1.
     */
    public static int ledBrightness;

    /**
     * Configures the LED's length, PWM port, and brightness
     * 
     * @param length     is the total number of LEDs you would like to operate
     * @param port       is the PWM port the LEDs are plugged into
     * @param brightness is how bright you would like you LEDs. Value between 0 and
     *                   1
     */
    public LedController(int length, int port, double brightness) {
        super(length, port, (int) brightness * 255);

        MathUtil.clamp(brightness, 0, 1);
        ledLength = length;
        ledPort = port;
        ledBrightness = (int) brightness * 255;

    }

    /**
     * This is how you tell the code which LEDs to light. You can have as many
     * sections as you would like, and they may overlap
     * 
     * @param title is the name of the section you are controlling. It can be
     *              whatever you desire, but must be consistent through your code
     *              per section
     * @param start is the LED ID that the section starts at. Untested whether this
     *              needs to be smaller than the end
     * @param end   is the LED ID that the section ends at. Untested whether this
     *              needs to be greater than the start
     */
    public void addSection(String title, int start, int end) {
        LedSectionConfig.addSection(title, start, end);
    }

    /**
     * This is required to update the LEDs. If this is not included in your periodic, the LEDs will not update.
     */
    public void updateLeds() {
        super.setData();
    }

    public int getLedBrightness() {
        return ledBrightness;
    }
}
