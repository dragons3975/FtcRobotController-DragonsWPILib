package org.firstinspires.ftc.teamcode;

public final class Constants {

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort2 = 1;
    }

    public static final class ConstantsDrivePID {
        public static final double kP = 0.005;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class ConstantsBras {
        public static final double coeficiant = 0.01;
    }

    public static final class ConstantsPince {

        public static final double etenduePince = 0.3;

        public static final double ouvreMax = 1;

        public static final double ouvreMin = 0;
    }

    public static final class ConstantsDrive {
        public static final double tacho1Tour = 8192.0 / 3.0;

        public static final double diametre = 10;

        public static final double distanceCalcul = (ConstantsDrive.tacho1Tour/(Math.PI * Constants.ConstantsDrive.diametre));


    }
    
}
