package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3130.robot.RobotMap;

import java.util.HashMap;
import java.util.Map;


public class WheelOfFortune extends SubsystemBase {

    //Create necessary objects
    private static ColorSensorV3 m_colorSensor;
    private static WPI_VictorSPX m_spinWheel;
    private static Solenoid m_wheelArm;

    //Create and define all standard data types needed
//    private SuppliedValueWidget colorWidget =
//            Shuffleboard.getTab("Kyber").addBoolean("Wheel Color", () -> true);

    private Map<String, String> fieldToTargetColorMap = new HashMap<String, String>();

    private double lastTimestamp;
    private boolean isChanged;
    private boolean isCounted;

    private String actualColor;

    private float deg = 0;
    private float sat = 0;
    private float brightness = 0;

    private Thread interrogator;
    private int cachedR = 0;
    private int cachedG = 0;
    private int cachedB = 0;

    private class InterrogationLoop implements Runnable {
		public void run() {
			try {
                // These three operations can run away waiting for i2c synchronization
                int r = m_colorSensor.getRed();
                int g = m_colorSensor.getGreen();
                int b = m_colorSensor.getBlue();
                // Now as we are back we can update the cache at once
                cachedR = r;
                cachedG = g;
                cachedB = b;
				Thread.sleep(20);
			} catch (InterruptedException e) {
				DriverStation.reportError(
						"Thread "+Thread.currentThread().getName()+" got interrupted",
						true);
				return;
			}
		}
	}

    public WheelOfFortune() {
        m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        m_spinWheel = new WPI_VictorSPX(RobotMap.CAN_WHEELOFFORTUNE);
        m_spinWheel.configFactoryDefault();
        m_spinWheel.setNeutralMode(NeutralMode.Brake);

        isChanged = false;

        m_wheelArm = new Solenoid(RobotMap.CAN_PNMMODULE, RobotMap.PNM_WHEELARM);

        m_wheelArm.set(false);

        //This is the Map for converting the fieldColor into targetColor, which can be used to clear a lot of confusion while making the algorithm
        fieldToTargetColorMap.put("Blue", "Red");
        fieldToTargetColorMap.put("Green", "Yellow");
        fieldToTargetColorMap.put("Red", "Blue");
        fieldToTargetColorMap.put("Yellow", "Green");

        actualColor = "Black"; // Initialize color tracker to black

        interrogator = new Thread(new InterrogationLoop(), "WOF interrogator");
        interrogator.start();

        DriverStation.reportWarning("WOF color sensor Init complete", false);
    }

    public String getTargetColor(String sourceColor, WheelOfFortune subsystem) {
        return subsystem.fieldToTargetColorMap.get(sourceColor);
    }

    public String determineColor() { //TODO: check with motor

        String possibleColor = this.detectHSB();

        if (!possibleColor.equals(actualColor)) {
            if (!isChanged) {
                lastTimestamp = Timer.getFPGATimestamp();
                isChanged = true;
                isCounted = false;
            } else {
                if (Timer.getFPGATimestamp() - lastTimestamp > .1 && !isCounted) {
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
    public String detectHSB() {
        float[] hsb = java.awt.Color.RGBtoHSB(cachedR, cachedG, cachedB, null);
        deg = hsb[0] * 360;
        sat = hsb[1];
        brightness = hsb[2];
        //Potential algorithm for rgb to hsb
        if (sat < 0.3 && brightness > 0.9) {
            return "White";
        } else if (brightness < 40.0) {
            return "Black";
        } else {
            if (deg < 60 || deg > 310) {
                return "Red";
            } else if (deg < 100) {
                return "Yellow";
            } else if (deg < 130) {
                return "Green";
            } else if (deg < 250) {
                return "Blue";
            } else {
                return "Bruh";
            }
        }
    }


    /**
     * Method for toggling wheel of fortune manipulator
     */
    public void toggleWheel() {
//        System.out.println("Wheel has toggled");
        m_wheelArm.set(!m_wheelArm.get());
    }

    /**
     * method for retracting wheel to be called in a command
     */
    public void retractWheel() {
//        System.out.println("Wheel has retracted");
        m_wheelArm.set(false);
    }

    public void motorSpin(double spin) {
        m_spinWheel.set(ControlMode.PercentOutput, spin);
    }

    public void outputToShuffleboard(WheelOfFortune subsystem) {
        SmartDashboard.putString("HSB Detected color", subsystem.detectHSB());
        SmartDashboard.putNumber("Hue Degree", subsystem.deg); //TODO: remove these
        SmartDashboard.putNumber("Saturation", subsystem.sat);
        SmartDashboard.putNumber("Brightness", subsystem.brightness);
    }

    @Override
    public void periodic() {
//        colorWidget.withProperties(Map.of("colorWhenTrue", getInstance().determineColor()));
    }

}
