package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "FTC2024-25Main_2")
public class Mecanum2 extends LinearOpMode {
    static final int CYCLE_MS = 50;     // period of each cycle
    public DcMotor[] wb = new DcMotor[4];
    public static int lf = 0;
    public static int lb = 1;
    public static int rb = 2;
    public static int rf = 3;
    public DcMotor  armMotor    = null; //the arm motor

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    public CRServo intake      = null; //the active intake servo
    public Servo wrist       = null; //the wrist servo
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;



//    float rulerpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        wb[lf] = hardwareMap.get(DcMotor.class, "lf");
        wb[rf] = hardwareMap.get(DcMotor.class, "rf");
        wb[lb] = hardwareMap.get(DcMotor.class, "lb");
        wb[rb] = hardwareMap.get(DcMotor.class, "rb");
        armMotor   = hardwareMap.get(DcMotor.class, "m0"); //the arm motor
        wb[rb].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[lf].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[lb].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[rf].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wb[rf].setDirection(DcMotor.Direction.REVERSE);
        wb[rb].setDirection(DcMotor.Direction.REVERSE);
        //wb[lf].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wb[lb].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wb[rb].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wb[rf].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake = hardwareMap.get(CRServo.class, "s0");
        wrist  = hardwareMap.get(Servo.class, "s1");

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);


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

        if (gamepad2.a) {
            intake.setPower(INTAKE_COLLECT);
        }
        else if (gamepad2.x) {
            intake.setPower(INTAKE_OFF);
        }
        else if (gamepad2.b) {
            intake.setPower(INTAKE_DEPOSIT);
        }



        if(gamepad2.right_bumper){
            /* This is the intaking/collecting arm position */
            armPosition = ARM_COLLECT;
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
        }

        else if (gamepad2.left_bumper){
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
            armPosition = ARM_CLEAR_BARRIER;
        }

        else if (gamepad2.y){
            /* This is the correct height to score the sample in the LOW BASKET */
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
        }

        else if (gamepad2.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad2.dpad_right){
            /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
            armPosition = ARM_SCORE_SPECIMEN;
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad2.dpad_up){
            /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
            armPosition = ARM_ATTACH_HANGING_HOOK;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        else if (gamepad2.dpad_down){
            /* this moves the arm down to lift the robot up once it has been hooked */
            armPosition = ARM_WINCH_ROBOT;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.update();


        double sin = 1 / Math.sqrt(2);
        double cos = 1 / Math.sqrt(2);
        double k = 1;

        double joyX = gamepad1.right_stick_x;
        double joyY = -gamepad1.right_stick_y;
        double trigR = gamepad1.left_trigger;
        double trigL = gamepad1.right_trigger;
        boolean bmpR = gamepad1.left_bumper;
        boolean bmpL = gamepad1.right_bumper;

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
