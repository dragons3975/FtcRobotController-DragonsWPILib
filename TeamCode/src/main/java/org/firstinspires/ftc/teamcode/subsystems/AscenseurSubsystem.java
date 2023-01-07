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
    private final DcMotor mMoteurAscenseurDroit;
    private final PIDController mPIDascenseurGauche = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);
    private final PIDController mPIDascenseurDroit = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);

    private boolean mPIDenabled = false;
    private double mPowerLeft = 0;
    private double mPowerRight = 0;

    private double mCalibrationLeft;
    private double mCalibrationRight;

    private DigitalChannel mDigitalInputLeft;
    private DigitalChannel mDigitalInputRight;



    public AscenseurSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mMoteurAscenseurGauche = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur gauche");
        mMoteurAscenseurDroit = mHardwareMap.get(DcMotor.class, "Moteur Ascenseur droit");

        mDigitalInputLeft = mHardwareMap.get(DigitalChannel.class, "LeftTactile");
        mDigitalInputRight = mHardwareMap.get(DigitalChannel.class, "RightTactile");

        mMoteurAscenseurGauche.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurAscenseurGauche.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mMoteurAscenseurDroit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMoteurAscenseurDroit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mPIDascenseurGauche.setTolerance(Constants.AscenseurPIDConstants.kToleranceAscenseur);
        mPIDascenseurGauche.setTolerance(Constants.AscenseurPIDConstants.kToleranceAscenseur);
    }


    @Override
    public void periodic() {
        if(mPIDenabled) {
            mPowerLeft = mPIDascenseurGauche.calculate(getAscenseur());
            mPowerRight = mPIDascenseurDroit.calculate(getAscenseur());
        }
            if(isLeftDown() && mPowerLeft < 0) {mMoteurAscenseurGauche.setPower(0);}
            else {
                mMoteurAscenseurGauche.setPower(mPowerLeft);
            }
            if(isRightDown() && mPowerRight < 0) {
                mMoteurAscenseurDroit.setPower(0);
            }
            else {mMoteurAscenseurDroit.setPower(mPowerRight);}
    }

    public double getAscenseur() {
        double getAscenseur = (mMoteurAscenseurGauche.getCurrentPosition() + mMoteurAscenseurDroit.getCurrentPosition())/2;
        return getAscenseur;
    }

    public void setSetPointAscenseur(double consigne) {
       mPIDascenseurGauche.setSetpoint(consigne+mCalibrationLeft);
       mPIDascenseurDroit.setSetpoint(consigne+mCalibrationRight);
        mPIDenabled = true;

    }

    public boolean atSetPointAscenseur() {
        return mPIDascenseurGauche.atSetpoint() && mPIDascenseurDroit.atSetpoint();
    }

    public boolean isLeftDown(){
    return mDigitalInputLeft.getState();
    }

    public boolean isRightDown() {
        return mDigitalInputRight.getState();
    }

    public void manualOveride(double power) {
        mPowerLeft = power;
        mPowerRight = power;
        mPIDenabled = false;
    }

    public void calibrate() {
        mCalibrationLeft = mMoteurAscenseurGauche.getCurrentPosition();
        mCalibrationRight = mMoteurAscenseurDroit.getCurrentPosition();
    }

    public void setPower(double power) {
        mMoteurAscenseurGauche.setPower(power);
        mMoteurAscenseurDroit.setPower(power);
    }

    public void stop() {
        mPowerLeft = 0;
        mPowerRight = 0;
        mPIDenabled = false;
    }

}

