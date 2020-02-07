package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3130.robot.RobotMap;

import java.util.HashMap;
import java.util.Map;


public class WheelOfFortune implements Subsystem {

    //Create necessary objects
    private static ColorSensorV3 m_colorSensor;
    private static ColorMatch m_colorMatcher;
    private static WPI_TalonSRX m_spinWheel;
    private static Map<String, String> fieldToTargetColorMap = new HashMap<String, String>();

    private static Solenoid m_wheelArm;


    //Create and define all standard data types needed
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    //This is the Map for converting the fieldColor into targetColor, which can be used to clear a lot of confusion while making the algorithm
    static {
        fieldToTargetColorMap.put("Cyan", "Red");
        fieldToTargetColorMap.put("Green", "Yellow");
        fieldToTargetColorMap.put("Red", "Cyan");
        fieldToTargetColorMap.put("Yellow", "Green");
    }

    private static double lastTimestamp;
    private static Timer timer;

    private static boolean isChanged;
    private static boolean isCounted;

    private static String actualColor = "No Color Yet";

    /**
     * The Singleton instance of this WheelOfFortune. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static WheelOfFortune INSTANCE = new WheelOfFortune();

    /**
     * Returns the Singleton instance of this WheelOfFortune. This static method
     * should be used -- {@code WheelOfFortune.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static WheelOfFortune getInstance() {
        return INSTANCE;
    }

    private WheelOfFortune() {
        m_colorSensor = new ColorSensorV3(i2cPort);

        m_spinWheel = new WPI_TalonSRX(RobotMap.CAN_WHEELOFFORTUNE);
        m_spinWheel.configFactoryDefault();

        isChanged = false;
        timer = new Timer();

        m_wheelArm = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_WHEELARM);

        m_wheelArm.set(false);

        timer.start();

    }

    public static String getTargetColor(String sourceColor) {
        return fieldToTargetColorMap.get(sourceColor);
    }

    public static int proximityCheck() {
        return m_colorSensor.getProximity();
    }

    public static String determineColor() { //TODO: check with motor

        String possibleColor = detectHSB();

        if (!possibleColor.equals(actualColor)) {
            if (!isChanged) {
                lastTimestamp = Timer.getFPGATimestamp();
                isChanged = true;
                isCounted = false;
            } else {
                if (Timer.getFPGATimestamp() - lastTimestamp > .2 && !isCounted) {
                    isCounted = true;
                    isChanged = false;
                    actualColor = possibleColor;
                }
            }
        } else {
            isChanged = false;
        }
        return actualColor;
    }

    /**
     * Run the color match algorithm on our detected color
     *
     * @return String name of the most likely color
     */
    public static String detectHSB() {
        int r = m_colorSensor.getRed();
        int g = m_colorSensor.getGreen();
        int b = m_colorSensor.getBlue();

        float hsb[] = java.awt.Color.RGBtoHSB(r, g, b, null);

        //Potential algorithm for rgb to hsb
        if (hsb[1] < 0.1 && hsb[2] > 0.9) {
            return "White";
        } else if (hsb[2] < 0.1) {
            return "Black";
        } else {
            float deg = hsb[0] * 360;
            float sat = hsb[1];
            float brightness = hsb[2];

            SmartDashboard.putNumber("Hue Degree", deg); //TODO: remove these
            SmartDashboard.putNumber("Saturation", sat);
            SmartDashboard.putNumber("Brightness", brightness);


            if ((deg >= 0 && deg < 80) || (deg > 310 && deg <= 360)) {
                return "Red";
            } else if (deg >= 110 && deg < 140) {
                return "Green";
            } else if (deg >= 135 && deg < 250) {
                return "Cyan";
            } else if (deg >= 83 && deg < 100) {
                return "Yellow";
            } else {
                System.out.println("bruh what color is this");
                return "Bruh";
            }
        }
    }

    /**
     * wheelArm(false) is when it is not deployed
     * wheelArm(true) is when it is deployed
     */


    //method for deploying wheel to be called in a command
    public static void deployWheel() {
        System.out.println("Wheel has deployed");
        m_wheelArm.set(true);
    }

    //method for retracting wheel to be called in a command
    public static void retractWheel() {
        System.out.println("Wheel has retracted");
        m_wheelArm.set(false);
    }


    public static void motorSpin(double spin) {
        m_spinWheel.set(spin);
    }

    public static void outputToSmartDashboard() {
        SmartDashboard.putString("HSB Detected color", detectHSB());
    }
}