package frc.team3130.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class WayPoints {

        //Fixed points
        public static final Translation2d kWasRed = new Translation2d(20, 115);
        public static final Translation2d kWasBlue = new Translation2d(185, 40);

        //TODO: Change all of these
        /** ------------------Galactic Search Challenge------ */
        public static final Translation2d path_A_Red1 = new Translation2d(90, 90);
        public static final Translation2d path_A_Red2 = new Translation2d(150, 60);
        public static final Translation2d path_A_Red3 = new Translation2d(180, 150);
        public static final Translation2d path_A_Red_end = new Translation2d(330, 150);

        public static final Translation2d path_A_Blue1 = new Translation2d(180, 30);
        public static final Translation2d path_A_Blue2 = new Translation2d(210, 120);
        public static final Translation2d path_A_Blue3 = new Translation2d(270, 90);
        public static final Translation2d path_A_Blue_end = new Translation2d(330, 90);

        public static final Translation2d path_B_Red1 = new Translation2d(90, 120);
        public static final Translation2d path_B_Red2 = new Translation2d(150, 60);
        public static final Translation2d path_B_Red3 = new Translation2d(210, 120);
        public static final Translation2d path_B_Red_end = new Translation2d(330, 120);

        public static final Translation2d path_B_Blue1 = new Translation2d(180, 60);
        public static final Translation2d path_B_Blue2 = new Translation2d(240, 120);
        public static final Translation2d path_B_Blue3 = new Translation2d(300, 60);
        public static final Translation2d path_B_Blue_end = new Translation2d(330, 60);

        //TODO: Change all of these points
        /**-------------------Barrel Racing Path----------------- */
        public static final Translation2d conjunction = new Translation2d(225, 90);
        public static final Translation2d[] barrel_points = {
                new Translation2d(120, 90),
                new Translation2d(155, 80),
                new Translation2d(172, 50),
                new Translation2d(150, 35),
                new Translation2d(82, 50),
                new Translation2d(155, 80),
                new Translation2d(255, 90),
                new Translation2d(244, 124),
                new Translation2d(220, 130),
                new Translation2d(255, 90),
                new Translation2d(263, 50),
                new Translation2d(300, 35),
                new Translation2d(330, 60),
                new Translation2d(300, 85),
                new Translation2d(255, 90),
                new Translation2d(160, 100),
                new Translation2d(60-29.5, 105),
        };

        /**-------------------salthom----------------- */
        public static final Translation2d[] salthom = {
                new Translation2d(90, 30),
                new Translation2d(120, 90),
                new Translation2d(180, 110),
                new Translation2d(250, 80),
                new Translation2d(270, 40),
                new Translation2d(300, 30),
                new Translation2d(315, 40),
                new Translation2d(335, 60),
                new Translation2d(315, 80),
                new Translation2d(300, 90),
                new Translation2d(285, 80),
                new Translation2d(270, 60),
                new Translation2d(255, 40),
                new Translation2d(255, 40),
                new Translation2d(180, 30),
                new Translation2d(150, 30),
                new Translation2d(100, 45),
                new Translation2d(75, 75),
                new Translation2d(60, 90)
        };

        /**-------------------Bounce Path----------------- */
        public static final Translation2d[] bounce_star1_1 = {new Translation2d(75, 100)};
        public static final Translation2d bounce_star1 = new Translation2d(90, 150);
        public static final Translation2d[] bounce_star2_1 = {
                new Translation2d(115, 80),
                new Translation2d(135, 45),
                new Translation2d(165, 45),
                new Translation2d(175, 55)
        };
        public static final Translation2d bounce_star2 = new Translation2d(180, 150);
        public static final Translation2d[] bounce_star3_1 = {
                new Translation2d(190, 45),
                new Translation2d(210, 35),
                new Translation2d(240, 35),
                new Translation2d(255, 40)
        };

        /** S path */
        public static final Translation2d[] DriveS = {
                new Translation2d(20, 10),
                new Translation2d(0, 20),
                new Translation2d(20, 30),
                new Translation2d(0, 40)
        };

}
