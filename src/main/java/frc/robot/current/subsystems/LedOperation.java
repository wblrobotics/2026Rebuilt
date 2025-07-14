package frc.robot.current.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.leds.LedColor;
import frc.robot.lib.leds.LedController;

public class LedOperation extends SubsystemBase {
  public static final LedController leds = new LedController(300, 3, .75);

  // Constants regarding manual LED states
  private static final String setSolid = "Solid";
  private static final String setTwoToneSolid = "Two Color Orange and Magenta";
  private static final String rainbow = "Rainbow";
  private static final String waveBlueGreen = "Wave Blue and Green";
  private static final String breathLightBlue = "Breath LightBlue";
  private static final String strobeRed = "Red Strobe";
  private static final String setColorBlack = "Black Light";
  private static final String fillGreen = "Fill pink";
  private static final String zipAqua = "Zip Aqua";
  private static final String wavePurple = "Wave Purple";
  private static final String colorTest = "Color Testing";
  private String manualLedState;

  private static final String red = "red";
  private static final String redOrange = "Red Orange";
  private static final String orange = "orange";
  private static final String gold = "gold";
  private static final String yellow = "yellow";
  private static final String yellowGreen = "yellow Green";
  private static final String lime = "lime";
  private static final String green = "green";
  private static final String aqua = "aqua";
  private static final String lightBlue = "light Blue";
  private static final String blue = "Blue";
  private static final String cornflower = "Cornflower";
  private static final String indigo = "indigo";
  private String ledColor;
  private LedColor color;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_color = new SendableChooser<>();

  // Constants regarding automatic LED states
  public ShuffleboardTab tab = Shuffleboard.getTab("Robot");
  public GenericEntry hueValue = tab.add("Hue Value", 0)
      .getEntry();

  public boolean automaticLED = false;

  public LedOperation() {
    leds.addSection("full", 0, 58);
    leds.addSection("front", 0, 27);
    leds.addSection("back", 27, 58);
    leds.addSection("top", 19, 34);
    leds.addSection("frontBottom", 0, 19);
    leds.addSection("backBottom", 34, 58);

    m_chooser.setDefaultOption("Solid", setSolid);
    m_chooser.addOption("Two Color Orange and Magenta", setTwoToneSolid);
    m_chooser.addOption("Solid Black", setColorBlack);
    m_chooser.addOption("Rainbow", rainbow);
    m_chooser.addOption("Wave Blue and Green", waveBlueGreen);
    m_chooser.addOption("Breath Light Blue", breathLightBlue);
    m_chooser.addOption("Red Strobe", strobeRed);
    m_chooser.addOption("Fill Green", fillGreen);
    m_chooser.addOption("Zip Aqua", zipAqua);
    m_chooser.addOption("Wave Purple", wavePurple);
    m_chooser.addOption("Color testing", colorTest);

    m_color.addOption("Red", red);
    m_color.addOption("", orange);
    m_color.addOption("", yellow);
    m_color.addOption("", yellowGreen);
    m_color.addOption("", green);
    m_color.addOption("", lightBlue);
    m_color.addOption("", blue);
    m_color.addOption("", cornflower);
    SmartDashboard.putData("Manual LED", m_chooser);
    SmartDashboard.putData("LEDColor", m_color);
    manualLedState = m_chooser.getSelected();
  }

  @Override
  public void periodic() {
    manualLedState = m_chooser.getSelected();
    ledColor = m_chooser.getSelected();
    automaticLED = false;

    robotStatus();

    leds.updateLeds();
  }

  public void robotStatus() {
    if (DriverStation.isEStopped()) {
      leds.strobe("full", LedColor.RED, 1);
    } else if (DriverStation.isAutonomousEnabled()) {
      leds.rainbow("full", 4);
    } else if (DriverStation.isTeleopEnabled()) {
      if (automaticLED) {
        updateState();
      } else {
        manualState();
      }
    } else {
      if (DriverStation.isDSAttached()) {
        leds.fill("full", LedColor.ORANGE, 2, 2, false);
      } else if (DriverStation.isFMSAttached()) {
        leds.solid("full", LedColor.ORANGE);
      } else {
        leds.breath("full", LedColor.ORANGE, 3);
      }
    }
  }

  public void updateState() {
    // switch (supersystemState.getMode()) {
    //   case CORAL:
    //     leds.solid("top", LedColor.BRIGHT_PURPLE);
    //     break;
    //   case ALGAE:
    //     leds.solid("top", LedColor.AQUA);
    // }

    // if (supersystemState.isElevatorGoingUp()) {
    //   leds.fill("frontBottom", LedColor.YELLOW, 1, 3, false);
    //   leds.fill("backBottom", LedColor.YELLOW, 1, 3, true);
    // } else if (supersystemState.isElevatorGoingDown()) {
    //   leds.fill("frontBottom", LedColor.YELLOW, 1, 3, true);
    //   leds.fill("backBottom", LedColor.YELLOW, 1, 3, false);
    // } else {
    //   if (supersystemState.getElevatorStatus()) {
    //     leds.breath("frontBottom", LedColor.GREEN, 2);
    //     leds.breath("backBottom", LedColor.GREEN, 2);
    //   } else {
    //     leds.solid("frontBottom", LedColor.RED);
    //     leds.solid("backBottom", LedColor.RED);
    //   }
    // }
  }

  /** Method to set the LEDs to different states during the match */
  public void manualState() {
    switch (ledColor) {
      case red:
        color = LedColor.RED;
        break;
      case redOrange:
        color = LedColor.RED_ORANGE;
        break;
      case orange:
        color = LedColor.ORANGE;
        break;
      default:
        color = LedColor.AQUA;
    }

    switch (manualLedState) {
      case setSolid:
        leds.solid("full", color);
        break;
      case setTwoToneSolid:
        leds.solidTwoColor("full", LedColor.ORANGE, LedColor.MAGENTA);
        break;
      case waveBlueGreen:
        leds.fade("full", LedColor.GREEN, LedColor.BLUE, 1, 3);
        break;
      case breathLightBlue:
        leds.breath("full", LedColor.LIGHT_BLUE, 3);
        break;
      case rainbow:
        leds.rainbow("full", 3);
        break;
      case strobeRed:
        leds.strobe("full", LedColor.RED, 1);
        break;
      case setColorBlack:
        leds.solid("full", LedColor.BLACK);
        break;
      case fillGreen:
        leds.fill("full", LedColor.GREEN, 1, 2, true);
        break;
      case zipAqua:
        leds.zip("full", LedColor.AQUA, 10, 1, 2, true);
        break;
      case wavePurple:
        leds.wave("full", LedColor.BRIGHT_PURPLE, 3);
      case colorTest:
        leds.colorTest("full", hueValue.getDouble(0));
        break;
      default:
        leds.rainbow("full", 3);
        break;
    }
  }
}
