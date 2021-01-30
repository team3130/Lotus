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
        DO_NOTHING,
    }

    public enum BeforeBarrelRacing {
        TRAVELING_TO_FIRST_POINT,
        TRAVELING_TO_SECOND_POINT,
        TRAVELING_TO_THIRD_POINT,
        DO_NOTHING,
    }

    public enum BeforeBouncePath {
        TRAVELING_TO_FIRST_POINT,
        TRAVELING_TO_SECOND_POINT,
        TRAVELING_TO_THIRD_POINT,
        DO_NOTHING,
    }

    public enum BeforeGalacticSearchA {
        TRAVELING_TO_FIRST_POINT,
        TRAVELING_TO_SECOND_POINT,
        TRAVELING_TO_THIRD_POINT,
        DO_NOTHING,
    }

    public enum BeforeGalacticSearchB {
        TRAVELING_TO_FIRST_POINT,
        TRAVELING_TO_SECOND_POINT,
        TRAVELING_TO_THIRD_POINT,
        DO_NOTHING,
    }

    public enum BeforeSlalomPath {
        TRAVELING_TO_FIRST_POINT,
        TRAVELING_TO_SECOND_POINT,
        TRAVELING_TO_THIRD_POINT,
        DO_NOTHING,
    }



    /**<p>making referance variables</p>*/
    /*
    private DriveS m_driveS = null;
    private BarrelRacing m_barrelRacing = null;
    private BouncePath m_bouncePath = null;
    private GalacticSearchA m_galacticSearchA = null;
    private GalacticSearchB m_galacticSearchB = null;
    private SlalomPath m_slalomPath = null;
     */

    /**<p>Makes an array list for a specific chooser</p>*/
    /*
    private SendableChooser<DriveS> m_driveS_Chooser;
    private SendableChooser<BarrelRacing> m_barrelRacing_Chooser;
    private SendableChooser<BouncePath> m_bouncePath_Chooser;
    private SendableChooser<GalacticSearchA> m_galacticSearchA_Chooser;
    private SendableChooser<GalacticSearchB> m_galacticSearchB_Chooser;
    private SendableChooser<SlalomPath> m_slalomPath_Chooser;
    */

    /** <p>Making an automatic option chooser</p> */
    private Optional<AutoOptionBase> autoOption_ = Optional.empty();

    /** <p>logs an initial point</p>*/
    private static Translation2d initialPoint;

    /** <p>Constructor for auto chooser</p>*/
    public AutoChooser() {

    }
}
