/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.nio.file.Path;
import java.sql.RowId;

import javax.tools.FileObject;
import javax.tools.ForwardingFileObject;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Simple Parking by encoders", group="Robot")
//@Disabled
public class ParkingByEncoders extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb  = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 5.52 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;
    static final int        DeegreErroRate          = 42;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          lf.getCurrentPosition(),
                          rf.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        left(DRIVE_SPEED, MetrToInches( (float) 0.5), 100);
        right(DRIVE_SPEED, MetrToInches( (float) 0.3), 100);
//        encoderDrive(DRIVE_SPEED, MetrToInches( (float) 0.4 ), MetrToInches( (float) 0.4 ), 100  );
        forward(DRIVE_SPEED, MetrToInches((float) 0.4), 100);
        left(DRIVE_SPEED, MetrToInches((float) 0.1), 100);
//        encoderDrive(DRIVE_SPEED, MetrToInches( (float) -0.3 ), MetrToInches( (float) -0.3 ), 100  );
        back(DRIVE_SPEED, MetrToInches((float) 0.3), 100);
        right(DRIVE_SPEED, MetrToInches((float) 1), 100);


        telemetry.update();
//        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = lf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            lf.setTargetPosition(newLeftTarget);
            rf.setTargetPosition(newRightTarget);
            lb.setTargetPosition(newLeftTarget);
            rb.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lf.setPower(Math.abs(speed));
            rf.setPower(Math.abs(speed));
            lb.setPower(Math.abs(speed));
            rb.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            lf.getCurrentPosition(), rf.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);

            // Turn off RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            sleep(250);   // optional pause after each move.
        }
    }
    public float MetrToInches(float Metr) {
        float inches = (float) (Metr * 39.4);
        return inches;
    }
    public float DegreesToInches(int Deegre) {
        float RoundLen = (float) (WHEEL_DIAMETER_INCHES * Math.PI);
        return RoundLen * Deegre / 360;
    }
    private void left(double speed, double PathInMetres, int timeout) {
        setMotorPowers(speed , -speed, speed, -speed, PathInMetres, timeout);
    }

    private void forward(double speed, double PathInMetres, int timeout) {
        setMotorPowers(speed, speed, speed, speed, PathInMetres, timeout);
        sleep(timeout);
    }

    private void back
            (double speed, double PathInMetres, int timeout) {
        setMotorPowers(-speed,-speed, -speed,-speed, PathInMetres, timeout);
    }


    private void right(double speed, double PathInMetres, int timeout) {
        setMotorPowers(-speed, speed, -speed, speed, PathInMetres, timeout );
    }

private void setMotorPowers(double lbPower, double lfPower, double rfPower, double rbPower,
                             double PathInMetres, double timeout) {
    int newTarget = lf.getCurrentPosition() + (int) (PathInMetres * COUNTS_PER_INCH);

    // Set target positions
    lf.setTargetPosition((int) (newTarget * lfPower / Math.abs(lfPower)));
    lb.setTargetPosition((int) (newTarget * lbPower / Math.abs(lbPower)));
    rf.setTargetPosition((int) (newTarget * rfPower / Math.abs(rfPower)));
    rb.setTargetPosition((int) (newTarget * rbPower / Math.abs(rbPower)));

    // Set motor modes
    lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // reset the timeout time and start motion.
    runtime.reset();

    // Set power levels
    lf.setPower(lfPower);
    rf.setPower(rfPower);
    lb.setPower(lbPower);
    rb.setPower(rbPower);

    // Loop while motors are busy
    while (opModeIsActive()  &&
                   (runtime.seconds() < timeout) && (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())) {
        telemetry.addData("Running to", " %7d :%7d", newTarget, newTarget);
        telemetry.addData("Currently at", " at %7d :%7d", lf.getCurrentPosition(), rf.getCurrentPosition());
        telemetry.update();
    }

    // Stop all motion
    lf.setPower(0);
    rf.setPower(0);
    lb.setPower(0);
    rb.setPower(0);

    // Reset motor modes
    lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    sleep((long) timeout);
}

}


