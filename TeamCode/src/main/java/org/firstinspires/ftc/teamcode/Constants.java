package org.firstinspires.ftc.teamcode;

public final class Constants {

    public static final class ConfigConstants {
        public static final int kRouge = 0;
        public static final int kBleu = 1;
        public static final int kGauche = 0;
        public static final int kDroite = 1;

    }


    public static final class AutonomousConstants {

        //POSITION 1, AVANCEMENT INITIAL
        public static final double kAvancementInitial = 65;

        public static final double kAvancementVersToileEloigne = 230;

        public static final double kAvancementVersToileProche = 110;

        public static final double kAvancementExtra1 = 120;

        public static final double kAvancementExtra2 = 110;

        public static final double kDeplacementExtraLateral = 50;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort2 = 1;
    }


    public static final class ConstantsBras {
        public static final double coeficiant = 0.01;

        public static final double kmax = 8500;

        public static final double kmaxExt = 47200;

    }

    public static final class ConstantsPince {

        public static final double InclinaisonHaut = 0.75;
        public static final double InclinaisonBas = 0.4;
        public static final double ouvreMax = 0.4;

        public static final double ouvreMin = 0.2;
    }

    public static final class ConstantsDrive {
        /*public static final double tacho1Tour = 8192.0 / 3.0;

        public static final double diametre = 10;*/

        public static final double distanceCalculx = 100.0 / 1510;//(ConstantsDrive.tacho1Tour/(Math.PI * Constants.ConstantsDrive.diametre));

        public static final double distanceCalculy = 50.0 / 825;
    }

    public static final class ConstantsDrivePID {
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;


        public static final double kPx = 0.03;
        public static final double kIx = 0;
        public static final double kDx = 0;


        public static final double kPy = 0.04;
        public static final double kIy = 0;
        public static final double kDy = 0;

    }

    public static final class VisionConstants {
        public static final String kTfodModelFile = "/sdcard/FIRST/tflitemodels/red_and_blue_camera_robot.tflite";
        public static final int kWidth = 320;
        public static final int kHeight = 240;
        public static final int kLimiteG = 250;

        public static final int kLimiteD = 650;

        public static final int kTeamPropGauche = 0;
        public static final int kTeamPropMilieu = 1;
        public static final int kTeamPropDroite = 2;

    }

    public static final class MaxSpeeds {
        public static final double kmaxZspeed = 0.5;

        public static final double kmaxXspeed = 0.5;

        public static final double kmaxYspeed = 0.5;

        public static final double kmaxRotationSpeed = 1;

        public static final double kmaxExtentionSpeed = 1;
    }


    
}
