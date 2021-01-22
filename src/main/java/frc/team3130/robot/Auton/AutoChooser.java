package frc.team3130.robot.Auton;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Optional;

public class AutoChooser {
    /** <p>An enum for the DriveS path</p>*/
    public enum BeforeDriveS {
        TRAVELING_TO_FIRST_POINT,
        TRAVELING_TO_SECOND_POINT,
        TRAVELING_TO_THIRD_POINT,
        TRAVELING_TO_FOURTH_POINT,
    }

    private DriveS m_driveS = null;

    /**<p>Makes an array list for a driveS specific chooser</p>*/
    private SendableChooser<DriveS> m_driveS_Chooser;

    /** <p>Making an automatic option chooser</p> */
    private Optional<AutoOptionBase> autoOption_ = Optional.empty();

    /** <p>logs an initial point</p>*/
    private static Translation2d initialPoint;

    /** <p>Constructor for auto chooser</p>*/
    public AutoChooser() {

    }

}
