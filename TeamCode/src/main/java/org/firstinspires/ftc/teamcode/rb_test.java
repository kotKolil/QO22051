package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="rb_test", group="Robot")
//@Disabled
public class rb_test extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private DcMotor balka_motor = null;
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.5;
    static final double TURN_SPEED = 0.7;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    // turn servo to grab element
    final double INTAKE_COLLECT    = -1.0;
    //turn servo to start pos and free element
    final double INTAKE_OFF        =  0.0;
    //hold element
    final double INTAKE_DEPOSIT    =  0.5;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    @Override
    public void runOpMode() {

         /* Define and initialize servos.*/
//        intake = hardwareMap.get(CRServo.class, "s0");
//        wrist  = hardwareMap.get(Servo.class, "s1");

        /* Make sure that the intake is off, and the wrist is folded in. */
//        intake.setPower(INTAKE_OFF);
//        wrist.setPosition(WRIST_FOLDED_IN);


        // Initialize the drive system variables.
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        //getting motor balka
//        balka_motor = hardwareMap.get(DcMotor.class, "m0");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // Set motor directions
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

//        balka_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        balka_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        rb.setPower(FORWARD_SPEED);
//        rf.setPower(rfPower);
//        lb.setPower(lbPower);
//        rb.setPower(rbPower);

//        forward(FORWARD_SPEED, 3000, 4);
//        telemetry.addLine("robot rode forward");
//        telemetry.update();
//        back(FORWARD_SPEED,3000, 4);
//        telemetry.addLine("robot rode back");
//
//        balka_motor.setTargetPosition(90);
//
//        grab_element();
//        sleep(1000);
//        ungrab_element();


        //back(FORWARD_SPEED, 1000);
//        wrist.setPosition(WRIST_FOLDED_OUT);
//        balka_motor.setTargetPosition((int) ARM_COLLECT);
//        sleep(1500);
//        intake.setPower(INTAKE_COLLECT);
//        sleep(100);
//        intake.setPower(INTAKE_DEPOSIT);
//        balka_motor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_LOW);
//        intake.setPower(INTAKE_OFF);



    }

    //function to grab element
    private void grab_element() {
        intake.setPower(INTAKE_COLLECT);
        intake.setPower(INTAKE_DEPOSIT);
    }

    private void ungrab_element() {
        intake.setPower(INTAKE_OFF);
    }

    // Defining functions for movement
    private void left(double speed, int time) {
        setMotorPowers(speed , -speed, -speed, speed);
        sleep(time);
        stopMotors();
    }

    private void forward(double speed, int time) {
        setMotorPowers(speed, speed, -speed, -speed);
        sleep(time);
        stopMotors();
    }

    private void back(double speed, int time) {
        setMotorPowers(-speed,-speed, speed,speed );
        sleep(time);
        stopMotors();
    }


    private void right(double speed, int time) {
        setMotorPowers(-speed, speed, speed, -speed );
        sleep(time);
        stopMotors();
    }

    private void setMotorPowers(double lbPower, double lfPower, double rfPower, double rbPower) {
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
}
