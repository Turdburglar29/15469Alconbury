package pedroPathing.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;
import javax.annotation.Nonnull;

@Config
public class FTC_DECODE_SUBSYSTEM {
    public static DcMotor intake;
    public static DcMotor spinner1;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private Servo releasespinner;
    private Servo sort1;
    private Servo sort2;


    public FTC_DECODE_SUBSYSTEM (@NonNull HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        spinner1 = hardwareMap.get(DcMotorEx.class, "spinner1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        releasespinner = hardwareMap.get(Servo.class, "releasespinner");
        sort1 = hardwareMap.get(Servo.class, "sort1");
        sort2 = hardwareMap.get(Servo.class, "sort2");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void intakerun() {
        intake.setPower(1);
    }

    public void intakestop() {
        intake.setPower(0);
    }




}



