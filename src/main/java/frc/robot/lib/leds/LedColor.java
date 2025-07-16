package frc.robot.lib.leds;

/**
 * This is a enum containing 37 LED colors in the HSV format.
 * Colors must be pulled from this file to be compatible with the White Bear
 * Lake LED Library.
 * 
 * If a color is not supported,
 * please use the colorTest() mode and comment on the GitHub repository with the
 * color recommendation (include web browser hue value)
 * Please keep in mind that these colors were tested with WS2812 light strands
 * and may show up differently on other light strands.
 * 
 * Hue values are divided by 2 because color pickers like Google are x/360, whereas WPILib is x/180
 * The value is set to the brightness of the LED, which is set in the LedController class.
 * If custom value is required add the value to the switch case in the value() method.
 */
public enum LedColor {
    LIGHT_RED(0, 192),
    SALMON(0, 125),
    RED(0, 125),
    RED_ORANGE(6 / 2, 125),
    PEACH(10 / 2, 125),
    ORANGE(10 / 2, 125),
    LIGHT_ORANGE(20 / 2, 125),
    GOLD(30 / 2, 125),
    LIGHT_YELLOW(50 / 2, 125),
    YELLOW(50 / 2, 125),
    SOFT_LIME(76 / 2, 125),
    YELLOW_GREEN(76 / 2, 125),
    EASTER_GREEN(90 / 2, 125),
    LIME(90 / 2, 125),
    LIGHT_GREEN(110 / 2, 125),
    GREEN(110 / 2, 125),
    AQUA(140 / 2, 125),
    SEA_GREEN(140 / 2, 125),
    SEA_BLUE(160 / 2, 125),
    LIGHT_BLUE(180 / 2, 125),
    TURQUOISE(180 / 2, 125),
    BABY_BLUE(200 / 2, 125),
    CORNFLOWER(200 / 2, 125),
    SKY_BLUE(215 / 2, 125),
    DARK_SKY_BLUE(240 / 2, 125),
    BLUE(240 / 2, 125),
    INDIGO(260 / 2, 125),
    LIGHT_PURPLE(260 / 2, 125),
    EASTER_PURPLE(276 / 2, 125),
    PURPLE(276 / 2, 125),
    BRIGHT_PURPLE(290 / 2, 125),
    LIGHT_PINK(336 / 2, 125),
    PINK(336 / 2, 125),
    ROSE(356 / 2, 125),
    MAGENTA(356 / 2, 125),
    BROWN(20 / 2, 70),
    WHITE(0, 0),
    BLACK(0, 0);

    private final int hue;
    private final int saturation;

    LedColor(int hue, int saturation) {
        this.hue = hue;
        this.saturation = saturation;
    }

    public int hues() {
        return hue;
    }

    public int saturation() {
        return saturation;
    }

    public int value() {
        switch (this) {
            case BROWN:
                return 60;
            case BLACK:
                return 0;
            default:
                if (LedController.ledBrightness > 0) {
                    return (int) LedController.ledBrightness;
                } else {
                    return 225;
                }
        }
    }
}
