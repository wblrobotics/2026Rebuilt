package frc.robot.current.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.leds.LedColor;
import frc.robot.lib.leds.LedController;

public class LedOperation extends SubsystemBase {
  public static final LedController leds = new LedController(240, 3, .75);

  // Constants regarding manual LED states
  private static final String setSolid = "Solid";
  private static final String setTwoToneSolid = "Two Color Orange and Magenta";
  private static final String rainbow = "Rainbow";
  private static final String fadeBlueGreen = "Wave Blue and Green";
  private static final String breathColor = "Breath";
  private static final String strobeColor = "Strobe";
  private static final String carnivalEasterGreenPurple = "Carnival";
  private static final String setColorBlack = "Black Light";
  private static final String fillColor = "Fill";
  private static final String zipColor = "Zip";
  private static final String waveColor = "Wave";
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
  private static final String seaBlue = "sea blue";
  private static final String lightBlue = "light Blue";
  private static final String skyBlue = "Sky Blue";
  private static final String blue = "Blue";
  private static final String cornflower = "Cornflower";
  private static final String indigo = "indigo";
  private static final String lightPurple = "Light Purple";
  private static final String purple = "Purple";
  private static final String lightPink = "Light Pink";
  private static final String rose = "rose";
  private static final String magenta = "Magenta";
  private static final String brown = "Brown";
  private static final String white = "White";
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
    leds.addSection("full", 0, 240);
    leds.addSection("right", 32, 80);
    leds.addSection("front", 81, 130);
    leds.addSection("left", 131, 191);
    leds.addSection("mechanismFrame", 32, 191);
    leds.addSection("underglow1", 0, 32);
    leds.addSection("underglow", 191, 230);

    m_chooser.setDefaultOption("Solid", setSolid);
    m_chooser.addOption("Two Color Solid", setTwoToneSolid);
    m_chooser.addOption("Solid Black", setColorBlack);
    m_chooser.addOption("Rainbow", rainbow);
    m_chooser.addOption("Fade Blue and Green", fadeBlueGreen);
    m_chooser.addOption("Breath", breathColor);
    m_chooser.addOption("Strobe", strobeColor);
    m_chooser.addOption("Carnival", carnivalEasterGreenPurple);
    m_chooser.addOption("Fill", fillColor);
    m_chooser.addOption("Zip", zipColor);
    m_chooser.addOption("Wave", waveColor);
    m_chooser.addOption("Color testing", colorTest);

    m_color.addOption("Red", red);
    m_color.addOption("Red-Orange", redOrange);
    m_color.addOption("Orange", orange);
    m_color.addOption("Gold", gold);
    m_color.addOption("Yellow", yellow);
    m_color.addOption("Yellow-Green", yellowGreen);
    m_color.addOption("Lime", lime);
    m_color.addOption("Green", green);
    m_color.addOption("Aqua", aqua);
    m_color.addOption("Sea Blue", seaBlue);
    m_color.addOption("Light Blue", lightBlue);
    m_color.addOption("Sky Blue", skyBlue);
    m_color.addOption("Blue", blue);
    m_color.addOption("Cornflower", cornflower);
    m_color.addOption("Indigo", indigo);
    m_color.addOption("Light Purple", lightPurple);
    m_color.addOption("purple", purple);
    m_color.addOption("Light Pink", lightPink);
    m_color.addOption("Rose", rose);
    m_color.addOption("Magenta", magenta);
    m_color.addOption("Brown", brown);
    m_color.addOption("White", white);

    SmartDashboard.putData("Manual LED", m_chooser);
    SmartDashboard.putData("LEDColor", m_color);
    manualLedState = m_chooser.getSelected();
    ledColor = m_color.getSelected();
  }

  @Override
  public void periodic() {
    manualLedState = m_chooser.getSelected();
    ledColor = m_color.getSelected();
    automaticLED = false;

    robotStatus();

    leds.updateLeds();
  }

  public void robotStatus() {
    if (DriverStation.isEStopped()) {
      leds.strobe("full", LedColor.RED, 1);
    } else if (DriverStation.isAutonomousEnabled()) {
      leds.rainbow("front", 4);
    } else if (DriverStation.isTeleopEnabled()) {
      if (automaticLED) {
        updateState();
      } else {
        manualState();
      }
    } else {
      if (DriverStation.isDSAttached()) {
        leds.fill("front", LedColor.ORANGE, 2, 2, false);
      } else if (DriverStation.isFMSAttached()) {
        leds.solid("front", LedColor.ORANGE);
      } else {
        leds.breath("front", LedColor.ORANGE, 2);
      }
    }

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Blue) {
        leds.solid("underglow", LedColor.BLUE);
        leds.solid("underglow1", LedColor.BLUE);
      } else {
        leds.solid("underglow", LedColor.RED);
        leds.solid("underglow1", LedColor.RED);
      }
    }
  }

  /** Method to set the LEDs automatically depending on the robots state */
  public void updateState() {

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
      case gold:
        color = LedColor.GOLD;
        break;
      case yellow:
        color = LedColor.YELLOW;
        break;
      case yellowGreen:
        color = LedColor.YELLOW_GREEN;
        break;
      case lime:
        color = LedColor.LIME;
        break;
      case green:
        color = LedColor.GREEN;
        break;
      case aqua:
        color = LedColor.AQUA;
        break;
      case seaBlue:
        color = LedColor.SEA_BLUE;
        break;
      case lightBlue:
        color = LedColor.LIGHT_BLUE;
        break;
      case skyBlue:
        color = LedColor.SKY_BLUE;
        break;
      case blue:
        color = LedColor.BLUE;
        break;
      case cornflower:
        color = LedColor.CORNFLOWER;
        break;
      case indigo:
        color = LedColor.INDIGO;
        break;
      case lightPurple:
        color = LedColor.LIGHT_PURPLE;
        break;
      case purple:
        color = LedColor.PURPLE;
        break;
      case lightPink:
        color = LedColor.LIGHT_PINK;
        break;
      case rose:
        color = LedColor.ROSE;
        break;
      case magenta:
        color = LedColor.MAGENTA;
        break;
      case brown:
        color = LedColor.BROWN;
        break;
      case white:
        color = LedColor.WHITE;
        break;
      default:
        color = LedColor.ORANGE;
        break;
    }

    switch (manualLedState) {
      case setSolid:
        leds.solid("mechanismFrame", color);
        break;
      case setTwoToneSolid:
        leds.solidTwoColor("mechanismFrame", LedColor.TURQUOISE, LedColor.PEACH);
        break;
      case fadeBlueGreen:
        leds.fade("mechanismFrame", LedColor.GREEN, LedColor.BLUE, 1, 3);
        break;
      case breathColor:
        leds.breath("mechanismFrame", color, 3);
        break;
      case rainbow:
        leds.rainbow("mechanismFrame", 3);
        break;
      case strobeColor:
        leds.strobe("mechanismFrame", color, 1);
        break;
      case carnivalEasterGreenPurple:
        leds.carnival("mechanismFrame", LedColor.EASTER_GREEN, LedColor.EASTER_PURPLE, 2, 4);
        break;
      case setColorBlack:
        leds.solid("mechanismFrame", LedColor.BLACK);
        break;
      case fillColor:
        leds.fill("mechanismFrame", color, 1, 2, true);
        break;
      case zipColor:
        leds.zip("mechanismFrame", color, 10, 1, 2, true);
        break;
      case waveColor:
        leds.wave("mechanismFrame", color, 3);
      case colorTest:
        leds.colorTest("mechanismFrame", hueValue.getDouble(0));
        break;
      default:
        leds.rainbow("mechanismFrame", 3);
        break;
    }
  }
}
