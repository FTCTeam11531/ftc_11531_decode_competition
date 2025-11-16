package org.firstinspires.ftc.teamcode.system.indexer;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Arrays;
import java.util.List;

public class Indexer {

//    public enum IndexerMode {
//
//    }

    // System OpMode
    private LinearOpMode opMode;

    // Intake Setting(s)


    // Define Hardware for subsystem
    private Servo indexLeft, indexRight, indexCenter;
    private List<Servo> listServoIndex;

    private NormalizedColorSensor indexColorLeft, indexColorRight, indexColorCenter;
    private List<NormalizedColorSensor> listSensorIndex;

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];

    // Constructor
    public Indexer(LinearOpMode opMode) { this.opMode = opMode; }

    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Indexer");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

        // Define and Initialize Motor(s)

        // Define and Initialize Servo(s)
        indexLeft = opMode.hardwareMap.get(Servo.class, RobotConstants.HardwareConfiguration.kLabelIndexServoLeft);
        indexRight = opMode.hardwareMap.get(Servo.class, RobotConstants.HardwareConfiguration.kLabelIndexServoRight);
        indexCenter = opMode.hardwareMap.get(Servo.class, RobotConstants.HardwareConfiguration.kLabelIndexServoCenter);

//        indexLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        indexRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        indexCenter.setDirection(DcMotorSimple.Direction.REVERSE);

        indexLeft.setPosition(RobotConstants.Indexer.Servo.KickoutLeft.kInit);
        indexRight.setPosition(RobotConstants.Indexer.Servo.KickoutRight.kInit);
        indexCenter.setPosition(RobotConstants.Indexer.Servo.KickoutCenter.kInit);

        // Define and Initialize Sensor(s)
        indexColorLeft = opMode.hardwareMap.get(NormalizedColorSensor.class, RobotConstants.HardwareConfiguration.kLabelIndexSensorLeft);
        indexColorRight = opMode.hardwareMap.get(NormalizedColorSensor.class, RobotConstants.HardwareConfiguration.kLabelIndexSensorRight);
        indexColorCenter = opMode.hardwareMap.get(NormalizedColorSensor.class, RobotConstants.HardwareConfiguration.kLabelIndexSensorCenter);

        listSensorIndex = Arrays.asList(indexColorLeft, indexColorRight, indexColorCenter);

        for (NormalizedColorSensor itemSensor : listSensorIndex) {

            // Turn on led light for sensor
            if (itemSensor instanceof SwitchableLight) {
                ((SwitchableLight)itemSensor).enableLight(true);
            }
        }

    }

    // ----------------------------------------------
    // Action Method(s)
    // ----------------------------------------------
    public void activateIndexer(String hardwareLabel, double setpoint) {

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelIndexServoLeft:
                indexLeft.setPosition(setpoint);
                break;

            case RobotConstants.HardwareConfiguration.kLabelIndexServoRight:
                indexRight.setPosition(setpoint);
                break;

            case RobotConstants.HardwareConfiguration.kLabelIndexServoCenter:
                indexCenter.setPosition(setpoint);
                break;
        }

    }

    public void deactivateIndexer(String hardwareLabel) {
        double setpoint = 0;

        activateIndexer(hardwareLabel, setpoint);
    }

    // ----------------------------------------------
    // Action Methods - Road Runner
    // ----------------------------------------------

    // Road Runner - Action - Activate Indexer
    public class ActionActivateIndexer implements Action {

        private String hardwareLabel;
        private double setpoint;

        // Action class constructor
        public ActionActivateIndexer(String hardwareLabel, double setpoint) {
            this.hardwareLabel = hardwareLabel;
            this.setpoint = setpoint;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            activateIndexer(hardwareLabel, setpoint);
            return false;
        }
    }

    public Action actionActivateIndexer(String hardwareLabel, double setpoint) {
        return new ActionActivateIndexer(hardwareLabel, setpoint);
    }


    // ----------------------------------------------
    // Get Method(s)
    // ----------------------------------------------
//    public double getIndexerPower(String hardwareLabel) {
//        double IndexerPower;
//
//        switch (hardwareLabel) {
//            case RobotConstants.HardwareConfiguration.kLabelIndexServoLeft:
//                IndexerPower = indexLeft.getPower();
//                break;
//
//            case RobotConstants.HardwareConfiguration.kLabelIndexServoRight:
//                IndexerPower = indexRight.getPower();
//                break;
//
//            case RobotConstants.HardwareConfiguration.kLabelIndexServoCenter:
//                IndexerPower = indexCenter.getPower();
//                break;
//
//            default:
//                IndexerPower = 0;
//        }
//
//        return IndexerPower;
//    }

    public NormalizedRGBA getNormalizedColors(String hardwareLabel) {
        NormalizedRGBA colors = null;

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelIndexServoLeft:
                colors = indexColorLeft.getNormalizedColors();
                break;

            case RobotConstants.HardwareConfiguration.kLabelIndexServoRight:
                colors = indexColorRight.getNormalizedColors();
                break;

            case RobotConstants.HardwareConfiguration.kLabelIndexServoCenter:
                colors = indexColorCenter.getNormalizedColors();

                break;

        }

        if (colors != null) {
            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);
        }

        return colors;
    }

    // ----------------------------------------------
    // Set Method(s)
    // ----------------------------------------------


}
