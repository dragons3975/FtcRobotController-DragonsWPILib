package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.dragonswpilib.math.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class AscenseurSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;

    private final DcMotor mMoteurAscenseurGauche;
    //private final DcMotor mMoteurAscenseurDroit;
    private final PIDController mPIDascenseurGauche = new PIDController(Constants.PIDascenseurConstants.kP, Constants.PIDascenseurConstants.kI, Constants.PIDascenseurConstants.kD);
    //private final PIDController mPIDascenseurDroit = new PIDController(Constants.PIDascenseurConstants.kP, Constants.PIDascenseurConstants.kI, Constants.PIDascenseurConstants.kD);

    private boolean mPIDenabled = false;
    private double mPowerLeft = 0;
    //private double mPowerRight = 0;

    private double mCalibrationLeft;
    //private double mCalibrationRight;

    private DigitalChannel mDigitalInputLeft;
    //private DigitalChannel mDigitalInputRight;



    public AscenseurSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mMoteurAscenseurGauche = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur gauche");
        //mMoteurAscenseurDroit = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur droit");

        mDigitalInputLeft = mHardwareMap.get(DigitalChannel.class, "LeftTactile");
        //mDigitalInputRight = mHardwareMap.get(DigitalChannel.class, "RightTactile");

        mMoteurAscenseurGauche.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurAscenseurGauche.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //mMoteurAscenseurDroit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mMoteurAscenseurDroit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mPIDascenseurGauche.setTolerance(Constants.PIDascenseurConstants.kTolerance);
        //mPIDascenseurDroit.setTolerance(Constants.PIDascenseurConstants.kTolerance);
    }


    @Override
    public void periodic() {
        if(mPIDenabled) {
            mPowerLeft = mPIDascenseurGauche.calculate(getAscenseurGauchePosition());
            //mPowerRight = mPIDascenseurDroit.calculate(getAscenseur());
        }
        if (isLeftDown() && mPowerLeft < 0) {
            mPowerLeft = 0;
        }
        mMoteurAscenseurGauche.setPower(mPowerLeft);
    }

    public double getAscenseurGauchePosition() {
        double tickGauche = mMoteurAscenseurGauche.getCurrentPosition();
        return tickGauche * Constants.AscenseurConstants.kTickParCm;
    }

    public void setSetPointAscenseur(double consigne) {
       mPIDascenseurGauche.setSetpoint(consigne + mCalibrationLeft);
       //mPIDascenseurDroit.setSetpoint(consigne+mCalibrationRight);
        mPIDenabled = true;
    }

    public boolean atSetPointAscenseur() {
        return mPIDascenseurGauche.atSetpoint(); //&& mPIDascenseurDroit.atSetpoint();
    }

    public boolean isLeftDown(){
    return mDigitalInputLeft.getState();
    }

    //public boolean isRightDown() {
    //    return mDigitalInputRight.getState();
    //}

    public void manualOveride(double power) {
        mPowerLeft = power;
        //mPowerRight = power;
        mPIDenabled = false;
    }

    public void calibrate() {
        mCalibrationLeft = mMoteurAscenseurGauche.getCurrentPosition();
        //mCalibrationRight = mMoteurAscenseurDroit.getCurrentPosition();
    }

    public void stop() {
        mPowerLeft = 0;
        //mPowerRight = 0;
        mPIDenabled = false;
    }

}

