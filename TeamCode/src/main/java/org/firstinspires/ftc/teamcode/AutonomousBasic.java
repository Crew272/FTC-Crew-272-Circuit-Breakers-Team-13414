package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonomous Full Routine", group = "Linear OpMode")
public class AutonomousBasic extends LinearOpMode {

    private RobotHardwareBasic robot = new RobotHardwareBasic();

    @Override
    public void runOpMode() {
        // Initialize hardware (includes odometry)
        robot.init(hardwareMap);

        // Reset odometry before the match starts
        robot.odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized & Odometry Reset");
        telemetry.addData("Starting X (mm)", robot.getCurrentPose().getX(DistanceUnit.MM));
        telemetry.addData("Starting Y (mm)", robot.getCurrentPose().getY(DistanceUnit.MM));
        telemetry.addData("Starting Heading (°)", robot.getCurrentPose().getHeading(AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Step 1: Move forward 24 inches
            //moveToPosition(24, 0, 0.5);

            // Step 2: Rotate 90 degrees clockwise
            //rotateToAngle(90, 0.4);

            // Step 3: Strafe right 36 inches
            moveToPosition(0, 36, 0.5);
        }
    }

    // Move to a specific position using odometry
    private void moveToPosition(double targetXInches, double targetYInches, double power) {
        double targetX = targetXInches * 25.4; // Convert inches to mm
        double targetY = targetYInches * 25.4;

        double Kp = robot.Kp;
        double Ki = 0.0005;
        double Kd = 0.002;

        double integralX = 0, integralY = 0;
        double previousErrorX = 0, previousErrorY = 0;
        final double POSITION_TOLERANCE_MM = 2.5; // **Reduce tolerance for better stopping**
        final double TIMEOUT_SECONDS = 5.0;
        final double INTEGRAL_BOUND = 100.0;

        double startTime = getRuntime();

        while (opModeIsActive()) {
            robot.odo.update(); // **Ensure fresh odometry data before calculations**
            Pose2D currentPose = robot.getCurrentPose();
            double currentX = currentPose.getX(DistanceUnit.MM);
            double currentY = currentPose.getY(DistanceUnit.MM);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // **Stop condition (tighter threshold)**
            if ((Math.abs(errorX) < POSITION_TOLERANCE_MM && Math.abs(errorY) < POSITION_TOLERANCE_MM)
                    || (getRuntime() - startTime > TIMEOUT_SECONDS)) {
                stopMotors();
                telemetry.addData("Status", "Arrived at target or timed out.");
                telemetry.update();
                break;
            }

            // **Prevent forward movement when strafing**
            if (targetXInches == 0) errorX = 0;
            if (targetYInches == 0) errorY = 0;

            // **Bound integral to prevent excessive correction**
            integralX = Math.max(-INTEGRAL_BOUND, Math.min(INTEGRAL_BOUND, integralX + errorX));
            integralY = Math.max(-INTEGRAL_BOUND, Math.min(INTEGRAL_BOUND, integralY + errorY));

            double derivativeX = errorX - previousErrorX;
            double derivativeY = errorY - previousErrorY;

            double powerX = (Kp * errorX) + (Ki * integralX) + (Kd * derivativeX);
            double powerY = (Kp * errorY) + (Ki * integralY) + (Kd * derivativeY);

            powerX = Math.max(-power, Math.min(power, powerX));
            powerY = Math.max(-power, Math.min(power, powerY));

            // **Apply correct strafing motor power**
            robot.frontLeft.setPower(powerX + powerY);
            robot.frontRight.setPower(powerX - powerY);
            robot.rearLeft.setPower(powerX - powerY);
            robot.rearRight.setPower(powerX + powerY);

            previousErrorX = errorX;
            previousErrorY = errorY;

            telemetry.addData("Target X (mm)", targetX);
            telemetry.addData("Target Y (mm)", targetY);
            telemetry.addData("Current X (mm)", currentX);
            telemetry.addData("Current Y (mm)", currentY);
            telemetry.addData("Error X (mm)", errorX);
            telemetry.addData("Error Y (mm)", errorY);
            telemetry.addData("Power X", powerX);
            telemetry.addData("Power Y", powerY);
            telemetry.addData("Time Elapsed", getRuntime() - startTime);
            telemetry.update();
        }
    }


    // Rotate to a specific angle using odometry
    private void rotateToAngle(double targetHeading, double power) {
        double Kp = 0.02;
        double Ki = 0.0;
        double Kd = 0.002;

        double error, previousError = 0;
        double integral = 0, derivative;
        final double HEADING_TOLERANCE = 2.0;
        final double TIMEOUT_SECONDS = 3.0;

        double startTime = getRuntime();

        while (opModeIsActive()) {
            robot.odo.update();
            double currentHeading = robot.getCurrentPose().getHeading(AngleUnit.DEGREES);

            error = targetHeading - currentHeading;
            integral += error;
            derivative = error - previousError;

            double turnPower = (Kp * error) + (Ki * integral) + (Kd * derivative);
            turnPower = Math.max(-power, Math.min(power, turnPower));

            robot.frontLeft.setPower(-turnPower);
            robot.frontRight.setPower(turnPower);
            robot.rearLeft.setPower(-turnPower);
            robot.rearRight.setPower(turnPower);

            previousError = error;

            if (Math.abs(error) < HEADING_TOLERANCE || (getRuntime() - startTime > TIMEOUT_SECONDS)) {
                stopMotors();
                telemetry.addData("Status", "Rotation complete.");
                telemetry.update();
                break;
            }

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }
    }

    private void stopMotors() {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        double stopTime = getRuntime();
        while (opModeIsActive() && getRuntime() - stopTime < 30) {
            telemetry.addData("Final Status", "Stopped - Holding telemetry for review.");
            telemetry.addData("Final X (mm)", robot.getCurrentPose().getX(DistanceUnit.MM));
            telemetry.addData("Final Y (mm)", robot.getCurrentPose().getY(DistanceUnit.MM));
            telemetry.addData("Final Heading (°)", robot.getCurrentPose().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
