package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp(name = "FTC2024-25Main")
public class Mecanum extends LinearOpMode {
    static final int CYCLE_MS = 50;     // period of each cycle
    public DcMotor[] wb = new DcMotor[4];
    public static int lf = 0;
    public static int lb = 1;
    public static int rb = 2;
    public static int rf = 3;
    //Servo   rukL;
    //Servo   rukR;
    //DcMotor Ruk;
    //Servo ruler1;
    //CRServo ruler2;
    //CRServo ruler3;
    //DcMotor utk;


    float rulerpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        wb[lf] = hardwareMap.get(DcMotor.class, "lf");
        wb[rf] = hardwareMap.get(DcMotor.class, "rf");
        wb[lb] = hardwareMap.get(DcMotor.class, "lb");
        wb[rb] = hardwareMap.get(DcMotor.class, "rb");
//        wb[lf] = hardwareMap.get(DcMotor.class, "left1");
//        wb[rf] = hardwareMap.get(DcMotor.class, "right1");
//        wb[lb] = hardwareMap.get(DcMotor.class, "left");
//        wb[rb] = hardwareMap.get(DcMotor.class, "right");
        wb[rb].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[lf].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[lb].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[rf].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[rf].setDirection(DcMotor.Direction.REVERSE);
        wb[rb].setDirection(DcMotor.Direction.REVERSE);
        wb[lf].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wb[lb].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wb[rb].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wb[rf].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  rukL = hardwareMap.get(Servo.class, "rukL");
      //  rukR = hardwareMap.get(Servo.class, "rukR");
      //  Ruk = hardwareMap.get(DcMotor.class, "Ruk");
      //  ruler1 = hardwareMap.get(Servo.class, "ruler1");
      //  ruler2 = hardwareMap.get(CRServo.class, "ruler2");
      //  ruler3 = hardwareMap.get(CRServo.class, "ruler3");
        

      //  ruler2.setDirection(CRServo.Direction.REVERSE);
      //  ruler3.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Init completed", null);

        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            WB_control();
        }

    }

    public void set_Power(double pwr_lf, double pwr_lb, double pwr_rb, double pwr_rf) {
        wb[lf].setPower((pwr_lf));
        wb[lb].setPower((pwr_lb));
        wb[rb].setPower((pwr_rb));
        wb[rf].setPower((pwr_rf));
    }

    void WB_control() {

        //if (gamepad2.dpad_up) {      //если клешня закрыта то открыть её по нажатию "x" на 2 джостике            )
        //    rukL.setPosition(1);
        //    rukR.setPosition(0);
        //    sleep(200);
        //                                        //                                                              )
        //} else if (gamepad2.dpad_down) {     //если клешня открыта то закрыть её по нажатию "x" на 2 джостике       } ruka
        //    rukL.setPosition(0.5);
        //    rukR.setPosition(0.5);
        //    sleep(200);
        //                                                 //                                                    )
        //}
        //utk.setpower(gamepad2.dpad_right);



        //Ruk.setPower(gamepad2.right_stick_y/2);

        //ruler1.setPosition(rulerpos);

        if (gamepad2.left_trigger >= 0.5) {
            if (rulerpos < 1) {
                rulerpos += 0.007;
            }
        }
        if (gamepad2.right_trigger >= 0.5) {
            if (rulerpos > 0) {
                rulerpos -= 0.007;
            }
        }
        if (rulerpos < 0) {
            rulerpos = 0;
        }
        if (rulerpos > 1) {
            rulerpos = 1;
        }
        //ruler2.setPower(-clip(gamepad2.left_stick_y, -1, 1));

        //if (gamepad2.y) {
        //    ruler3.setPower(1);
        //}
        //if (gamepad2.a) {
        //    ruler3.setPower(-1);
        //}
        //if (gamepad2.b){
        //    ruler3.setPower(0.02);
        //}

        double sin = 1 / Math.sqrt(2);
        double cos = 1 / Math.sqrt(2);
        double k = 1;

        double joyX = gamepad1.right_stick_x;
        double joyY = -gamepad1.right_stick_y;
        double trigL = gamepad1.left_trigger;
        double trigR = gamepad1.right_trigger;
        boolean bmpL = gamepad1.left_bumper;
        boolean bmpR = gamepad1.right_bumper;

        double targetX;
        double targetY;
        double turn;

        turn = (trigL - trigR) / 2;
        if (bmpL)
            turn = 1;
        if (bmpR)
            turn = -1;
        if (gamepad1.dpad_down)
            k = 0.6;


        //Preparing
        double len = joyX * joyX + joyY * joyY;
        targetY = cos * joyY - sin * joyX;
        targetX = cos * joyX + sin * joyY;
        turn *= k;

        //Extra_power
        if (Math.abs(targetX) >= Math.abs(targetY) && targetX != 0) {
            targetY /= Math.abs(targetX);
            targetX /= Math.abs(targetX);
        } else if (Math.abs(targetX) <= Math.abs(targetY) && targetY != 0) {
            targetY /= Math.abs(targetY);
            targetX /= Math.abs(targetY);
        }
        targetX *= len * k;
        targetY *= len * k;

        //Output
        if (len < 0.002) {
            set_Power(
                    turn,
                    turn,
                    -(turn),
                    -(turn));
            return;
        }
        set_Power(
                (targetX + turn),
                (targetY + turn),
                (targetX - turn),
                (targetY - turn)
        );
    }
}
