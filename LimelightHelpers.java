//LimelightHelpers v1.14 (REQUIRES LLOS 2026.0 OR LATER)

package frc.robot;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.Arrays;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.concurrent.ConcurrentHashMap;
import edu.wpi.first.net.PortForwarder;

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
public class LimelightHelpers {

    private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

    /**
     * Represents a Color/Retroreflective Target Result extracted from JSON Output
     */
    public static class LimelightTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private  double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;
        
        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }

    }

    /**
     * Represents an AprilTag/Fiducial Target Result extracted from JSON Output
     */
    public static class LimelightTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }
        
        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;
        
        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    /**
     * Represents a Barcode Target Result extracted from JSON Output
     */
    public static class LimelightTarget_Barcode {

        /**
         * Barcode family type (e.g. "QR", "DataMatrix", etc.)
         */
        @JsonProperty("fam")
        public String family;

        /**
         * Gets the decoded data content of the barcode
         */
        @JsonProperty("data") 
        public String data;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("pts")
        public double[][] corners;

        public LimelightTarget_Barcode() {
        }

        public String getFamily() {
            return family;
        }
    }

    /**
     * Represents a Neural Classifier Pipeline Result extracted from JSON Output
     */
    public static class LimelightTarget_Classifier {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("zone")
        public double zone;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public  LimelightTarget_Classifier() {
        }
    }

    /**
     * Represents a Neural Detector Pipeline Result extracted from JSON Output
     */
    public static class LimelightTarget_Detector {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        public LimelightTarget_Detector() {
        }
    }

    /**
     * Represents hardware statistics from the Limelight.
     */
    public static class HardwareReport {
        @JsonProperty("cid")
        public String cameraId;

        @JsonProperty("cpu")
        public double cpuUsage;

        @JsonProperty("dfree")
        public double diskFree;

        @JsonProperty("dtot")
        public double diskTotal;

        @JsonProperty("ram")
        public double ramUsage;

        @JsonProperty("temp")
        public double temperature;

        public HardwareReport() {
        }
    }

    /**
     * Represents IMU data from the JSON results.
     */
    public static class IMUResults {
        @JsonProperty("data")
        public double[] data;

        @JsonProperty("quat")
        public double[] quaternion;

        @JsonProperty("yaw")
        public double yaw;

        // Parsed from data array
        public double robotYaw;
        public double roll;
        public double pitch;
        public double rawYaw;
        public double gyroZ;
        public double gyroX;
        public double gyroY;
        public double accelZ;
        public double accelX;
        public double accelY;

        public IMUResults() {
            data = new double[0];
            quaternion = new double[4];
        }

        public void parseDataArray() {
            if (data != null && data.length >= 10) {
                robotYaw = data[0];
                roll = data[1];
                pitch = data[2];
                rawYaw = data[3];
                gyroZ = data[4];
                gyroX = data[5];
                gyroY = data[6];
                accelZ = data[7];
                accelX = data[8];
                accelY = data[9];
            }
        }
    }

    /**
     * Represents capture rewind buffer statistics.
     */
    public static class RewindStats {
        @JsonProperty("bufferUsage")
        public double bufferUsage;

        @JsonProperty("enabled")
        public int enabled;

        @JsonProperty("flushing")
        public int flushing;

        @JsonProperty("frameCount")
        public int frameCount;

        @JsonProperty("latpen")
        public int latencyPenalty;

        @JsonProperty("storedSeconds")
        public double storedSeconds;

        public RewindStats() {
        }
    }

    /**
     * Limelight Results object, parsed from a Limelight's JSON results output.
     */
    public static class LimelightResults {
        
        public String error;
        
        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        public double latency_jsonParse;

        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("ts_nt")
        public long timestamp_nt;

        @JsonProperty("ts_sys")
        public long timestamp_sys;

        @JsonProperty("ts_us")
        public long timestamp_us;

        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("pTYPE")
        public String pipelineType;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txnc")
        public double tx_nocrosshair;

        @JsonProperty("tync")
        public double ty_nocrosshair;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("botpose_tagcount")
        public double botpose_tagcount;
       
        @JsonProperty("botpose_span")
        public double botpose_span;
       
        @JsonProperty("botpose_avgdist")
        public double botpose_avgdist;
       
        @JsonProperty("botpose_avgarea")
        public double botpose_avgarea;

        @JsonProperty("botpose_orb")
        public double[] botpose_orb;

        @JsonProperty("botpose_orb_wpiblue")
        public double[] botpose_orb_wpiblue;

        @JsonProperty("botpose_orb_wpired")
        public double[] botpose_orb_wpired;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        @JsonProperty("hw")
        public HardwareReport hardware;

        @JsonProperty("imu")
        public IMUResults imuResults;

        @JsonProperty("rewind")
        public RewindStats rewindStats;

        @JsonProperty("PythonOut")
        public double[] pythonOutput;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }
    
        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }
    
        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }
    
        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }
    
        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @JsonProperty("Retro")
        public LimelightTarget_Retro[] targets_Retro;

        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("Classifier")
        public LimelightTarget_Classifier[] targets_Classifier;

        @JsonProperty("Detector")
        public LimelightTarget_Detector[] targets_Detector;

        @JsonProperty("Barcode")
        public LimelightTarget_Barcode[] targets_Barcode;

        public LimelightResults() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            botpose_orb = new double[6];
            botpose_orb_wpiblue = new double[6];
            botpose_orb_wpired = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            targets_Classifier = new LimelightTarget_Classifier[0];
            targets_Detector = new LimelightTarget_Detector[0];
            targets_Barcode = new LimelightTarget_Barcode[0];
            pythonOutput = new double[0];
            pipelineType = "";
        }


    }

    /**
     * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
     */
    public static class RawFiducial {
        public int id = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double distToCamera = 0;
        public double distToRobot = 0;
        public double ambiguity = 0;


        public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            RawFiducial other = (RawFiducial) obj;
            return id == other.id &&
                Double.compare(txnc, other.txnc) == 0 &&
                Double.compare(tync, other.tync) == 0 &&
                Double.compare(ta, other.ta) == 0 &&
                Double.compare(distToCamera, other.distToCamera) == 0 &&
                Double.compare(distToRobot, other.distToRobot) == 0 &&
                Double.compare(ambiguity, other.ambiguity) == 0;
        }

    }

    /**
     * Represents a Limelight Raw Target/Contour result from Limelight's NetworkTables output.
     */
    public static class RawTarget {
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;

        public RawTarget(double txnc, double tync, double ta) {
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            RawTarget other = (RawTarget) obj;
            return Double.compare(txnc, other.txnc) == 0 &&
                Double.compare(tync, other.tync) == 0 &&
                Double.compare(ta, other.ta) == 0;
        }
    }

    /**
     * Represents a Limelight Raw Neural Detector result from Limelight's NetworkTables output.
     */
    public static class RawDetection {
        public int classId = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double corner0_X = 0;
        public double corner0_Y = 0;
        public double corner1_X = 0;
        public double corner1_Y = 0;
        public double corner2_X = 0;
        public double corner2_Y = 0;
        public double corner3_X = 0;
        public double corner3_Y = 0;


        public RawDetection(int classId, double txnc, double tync, double ta, 
            double corner0_X, double corner0_Y, 
            double corner1_X, double corner1_Y, 
            double corner2_X, double corner2_Y, 
            double corner3_X, double corner3_Y ) {
            this.classId = classId;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.corner0_X = corner0_X;
            this.corner0_Y = corner0_Y;
            this.corner1_X = corner1_X;
            this.corner1_Y = corner1_Y;
            this.corner2_X = corner2_X;
            this.corner2_Y = corner2_Y;
            this.corner3_X = corner3_X;
            this.corner3_Y = corner3_Y;
        }
    }
    
    /**
     * Represents a 3D Pose Estimate.
     */
    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;

        public RawFiducial[] rawFiducials; 
        public boolean isMegaTag2;

        /**
         * Instantiates a PoseEstimate object with default values
         */
        public PoseEstimate() {
            this.pose = new Pose2d();
            this.timestampSeconds = 0;
            this.latency = 0;
            this.tagCount = 0;
            this.tagSpan = 0;
            this.avgTagDist = 0;
            this.avgTagArea = 0;
            this.rawFiducials = new RawFiducial[]{};
            this.isMegaTag2 = false;
        }

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency, 
            int tagCount, double tagSpan, double avgTagDist, 
            double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {

            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
            this.isMegaTag2 = isMegaTag2;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            PoseEstimate that = (PoseEstimate) obj;
            // We don't compare the timestampSeconds as it isn't relevant for equality and makes
            // unit testing harder
            return Double.compare(that.latency, latency) == 0
                && tagCount == that.tagCount
                && Double.compare(that.tagSpan, tagSpan) == 0
                && Double.compare(that.avgTagDist, avgTagDist) == 0
                && Double.compare(that.avgTagArea, avgTagArea) == 0
                && pose.equals(that.pose)
                && Arrays.equals(rawFiducials, that.rawFiducials);
        }

    }

    /**
     * Encapsulates the state of an internal Limelight IMU.
     */
    public static class IMUData {
        public double robotYaw = 0.0;
        public double Roll = 0.0;
        public double Pitch = 0.0;
        public double Yaw = 0.0;
        public double gyroX = 0.0;
        public double gyroY = 0.0;
        public double gyroZ = 0.0;
        public double accelX = 0.0;
        public double accelY = 0.0;
        public double accelZ = 0.0;

        public IMUData() {}

        public IMUData(double[] imuData) {
            if (imuData != null && imuData.length >= 10) {
                this.robotYaw = imuData[0];
                this.Roll = imuData[1];
                this.Pitch = imuData[2];
                this.Yaw = imuData[3];
                this.gyroX = imuData[4];
                this.gyroY = imuData[5];
                this.gyroZ = imuData[6];
                this.accelX = imuData[7];
                this.accelY = imuData[8];
                this.accelZ = imuData[9];
            }
        }
    }


    private static ObjectMapper mapper;

    /**
     * Print JSON Parse time to the console in milliseconds
     */
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if ("".equals(name) || name == null) {
            return "limelight";
        }
        return name;
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose3d object.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose3d object representing the pose, or empty Pose3d if invalid data
     */
    public static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose2d object.
     * Uses only x, y, and yaw components, ignoring z, roll, and pitch.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose2d object representing the pose, or empty Pose2d if invalid data
     */
    public static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    /**
     * Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     * 
     * @param pose The Pose3d object to convert
     * @return A 6-element array containing [x, y, z, roll, pitch, yaw]
     */
    public static double[] pose3dToArray(Pose3d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = pose.getTranslation().getZ();
        result[3] = Units.radiansToDegrees(pose.getRotation().getX());
        result[4] = Units.radiansToDegrees(pose.getRotation().getY());
        result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
        return result;
    }

    /**
     * Converts a Pose2d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     * Note: z, roll, and pitch will be 0 since Pose2d only contains x, y, and yaw.
     * 
     * @param pose The Pose2d object to convert
     * @return A 6-element array containing [x, y, 0, 0, 0, yaw]
     */
    public static double[] pose2dToArray(Pose2d pose) {
        double[] result = new double[6];
        result[0] = pose.getTranslation().getX();
        result[1] = pose.getTranslation().getY();
        result[2] = 0;
        result[3] = Units.radiansToDegrees(0);
        result[4] = Units.radiansToDegrees(0);
        result[5] = Units.radiansToDegrees(pose.getRotation().getRadians());
        return result;
    }

    private static double extractArrayEntry(double[] inData, int position){
        if(inData.length < position+1)
        {
            return 0;
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, entryName);
        
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;
        
        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return new PoseEstimate();
        }
    
        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int)extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);
        
        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
    
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
        RawFiducial[] rawFiducials;

        if (poseArray.length != expectedTotalVals) {
            // Array size mismatch - return empty array instead of null-filled array
            rawFiducials = new RawFiducial[0];
        } else {
            rawFiducials = new RawFiducial[tagCount];
            for(int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int)poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }
    
        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
    }

    /**
     * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
     * 
     * @param limelightName Name/identifier of the Limelight
     * @return Array of RawFiducial objects containing detection details
     */
    public static RawFiducial[] getRawFiducials(String limelightName) {
        var entry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "rawfiducials");
        var rawFiducialArray = entry.getDoubleArray(new double[0]);
        int valsPerEntry = 7;
        if (rawFiducialArray.length % valsPerEntry != 0) {
            return new RawFiducial[0];
        }
    
        int numFiducials = rawFiducialArray.length / valsPerEntry;
        RawFiducial[] rawFiducials = new RawFiducial[numFiducials];
    
        for (int i = 0; i < numFiducials; i++) {
            int baseIndex = i * valsPerEntry;
            int id = (int) extractArrayEntry(rawFiducialArray, baseIndex);
            double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
            double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
            double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
            double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
            double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
            double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);
            
            rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
        }
    
        return rawFiducials;
    }

    /**
     * Gets the latest raw neural detector results from NetworkTables
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of RawDetection objects containing detection details
     */
    public static RawDetection[] getRawDetections(String limelightName) {
        var entry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "rawdetections");
        var rawDetectionArray = entry.getDoubleArray(new double[0]);
        int valsPerEntry = 12;
        if (rawDetectionArray.length % valsPerEntry != 0) {
            return new RawDetection[0];
        }
    
        int numDetections = rawDetectionArray.length / valsPerEntry;
        RawDetection[] rawDetections = new RawDetection[numDetections];
    
        for (int i = 0; i < numDetections; i++) {
            int baseIndex = i * valsPerEntry; // Starting index for this detection's data
            int classId = (int) extractArrayEntry(rawDetectionArray, baseIndex);
            double txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1);
            double tync = extractArrayEntry(rawDetectionArray, baseIndex + 2);
            double ta = extractArrayEntry(rawDetectionArray, baseIndex + 3);
            double corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4);
            double corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5);
            double corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6);
            double corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7);
            double corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8);
            double corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9);
            double corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10);
            double corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11);
            
            rawDetections[i] = new RawDetection(classId, txnc, tync, ta, corner0_X, corner0_Y, corner1_X, corner1_Y, corner2_X, corner2_Y, corner3_X, corner3_Y);
        }
    
        return rawDetections;
    }

    /**
     * Gets the raw target contours from NetworkTables.
     * Returns ungrouped contours in normalized screen space (-1 to 1).
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of RawTarget objects containing up to 3 contours
     */
    public static RawTarget[] getRawTargets(String limelightName) {
        var entry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "rawtargets");
        var rawTargetArray = entry.getDoubleArray(new double[0]);
        int valsPerEntry = 3;
        if (rawTargetArray.length % valsPerEntry != 0) {
            return new RawTarget[0];
        }

        int numTargets = rawTargetArray.length / valsPerEntry;
        RawTarget[] rawTargets = new RawTarget[numTargets];

        for (int i = 0; i < numTargets; i++) {
            int baseIndex = i * valsPerEntry;
            double txnc = extractArrayEntry(rawTargetArray, baseIndex);
            double tync = extractArrayEntry(rawTargetArray, baseIndex + 1);
            double ta = extractArrayEntry(rawTargetArray, baseIndex + 2);

            rawTargets[i] = new RawTarget(txnc, tync, ta);
        }

        return rawTargets;
    }

    /**
     * Gets the corner coordinates of detected targets from NetworkTables.
     * Requires "send contours" to be enabled in the Limelight Output tab.
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of doubles containing corner coordinates [x0, y0, x1, y1, ...]
     */
    public static double[] getCornerCoordinates(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tcornxy");
    }

    /**
     * Prints detailed information about a PoseEstimate to standard output.
     * Includes timestamp, latency, tag count, tag span, average tag distance,
     * average tag area, and detailed information about each detected fiducial.
     *
     * @param pose The PoseEstimate object to print. If null, prints "No PoseEstimate available."
     */
    public static void printPoseEstimate(PoseEstimate pose) {
        if (pose == null) {
            System.out.println("No PoseEstimate available.");
            return;
        }
    
        System.out.printf("Pose Estimate Information:%n");
        System.out.printf("Timestamp (Seconds): %.3f%n", pose.timestampSeconds);
        System.out.printf("Latency: %.3f ms%n", pose.latency);
        System.out.printf("Tag Count: %d%n", pose.tagCount);
        System.out.printf("Tag Span: %.2f meters%n", pose.tagSpan);
        System.out.printf("Average Tag Distance: %.2f meters%n", pose.avgTagDist);
        System.out.printf("Average Tag Area: %.2f%% of image%n", pose.avgTagArea);
        System.out.printf("Is MegaTag2: %b%n", pose.isMegaTag2);
        System.out.println();
    
        if (pose.rawFiducials == null || pose.rawFiducials.length == 0) {
            System.out.println("No RawFiducials data available.");
            return;
        }
    
        System.out.println("Raw Fiducials Details:");
        for (int i = 0; i < pose.rawFiducials.length; i++) {
            RawFiducial fiducial = pose.rawFiducials[i];
            System.out.printf(" Fiducial #%d:%n", i + 1);
            System.out.printf("  ID: %d%n", fiducial.id);
            System.out.printf("  TXNC: %.2f%n", fiducial.txnc);
            System.out.printf("  TYNC: %.2f%n", fiducial.tync);
            System.out.printf("  TA: %.2f%n", fiducial.ta);
            System.out.printf("  Distance to Camera: %.2f meters%n", fiducial.distToCamera);
            System.out.printf("  Distance to Robot: %.2f meters%n", fiducial.distToRobot);
            System.out.printf("  Ambiguity: %.2f%n", fiducial.ambiguity);
            System.out.println();
        }
    }

    public static Boolean validPoseEstimate(PoseEstimate pose) {
        return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static void Flush() {
        NetworkTableInstance.getDefault().flush();
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(key, k -> {
            NetworkTable table = getLimelightNTTable(tableName);
            return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
    }
    
    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }


    public static String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    public static String[] getLimelightNTStringArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getStringArray(new String[0]);
    }


    /////

    /**
     * Does the Limelight have a valid target?
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return True if a valid target is present, false otherwise
     */
    public static boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    /**
     * Gets the horizontal offset from the principal pixel/point to the target in degrees.  This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    public static double getTXNC(String limelightName) {
        return getLimelightNTDouble(limelightName, "txnc");
    }

    /**
     * Gets the vertical offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    public static double getTYNC(String limelightName) {
        return getLimelightNTDouble(limelightName, "tync");
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     * @param limelightName Name of the Limelight camera ("" for default) 
     * @return Target area percentage (0-100)
     */
    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    /**
     * T2D is an array that contains several targeting metrcis
     * @param limelightName Name of the Limelight camera
     * @return Array containing  [targetValid, targetCount, targetLatency, captureLatency, tx, ty, txnc, tync, ta, tid, targetClassIndexDetector, 
     * targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees]
     */
    public static double[] getT2DArray(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "t2d");
    }
    
    /**
     * Gets the number of targets currently detected.
     * @param limelightName Name of the Limelight camera
     * @return Number of detected targets
     */
    public static int getTargetCount(String limelightName) {
      double[] t2d = getT2DArray(limelightName);
      if(t2d.length == 17)
      {
        return (int)t2d[1];
      }
      return 0;
    }

    /**
     * Gets the classifier class index from the currently running neural classifier pipeline
     * @param limelightName Name of the Limelight camera
     * @return Class index from classifier pipeline
     */
    public static int getClassifierClassIndex (String limelightName) {
    double[] t2d = getT2DArray(limelightName);
      if(t2d.length == 17)
      {
        return (int)t2d[11];
      }
      return 0;
    }

    /**
     * Gets the detector class index from the primary result of the currently running neural detector pipeline.
     * @param limelightName Name of the Limelight camera
     * @return Class index from detector pipeline
     */
    public static int getDetectorClassIndex (String limelightName) {
     double[] t2d = getT2DArray(limelightName);
      if(t2d.length == 17)
      {
        return (int)t2d[10];
      }
      return 0;
    }

    /**
     * Gets the current neural classifier result class name.
     * @param limelightName Name of the Limelight camera
     * @return Class name string from classifier pipeline
     */
    public static String getClassifierClass (String limelightName) {
        return getLimelightNTString(limelightName, "tcclass");
    }

    /**
     * Gets the primary neural detector result class name.
     * @param limelightName Name of the Limelight camera
     * @return Class name string from detector pipeline
     */
    public static String getDetectorClass (String limelightName) {
        return getLimelightNTString(limelightName, "tdclass");
    }

    /**
     * Gets the pipeline's processing latency contribution.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline latency in milliseconds
     */
    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    /**
     * Gets the capture latency.
     * @param limelightName Name of the Limelight camera
     * @return Capture latency in milliseconds
     */
    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    /**
     * Gets the active pipeline index.
     * @param limelightName Name of the Limelight camera
     * @return Current pipeline index (0-9)
     */
    public static double getCurrentPipelineIndex(String limelightName) {
        return getLimelightNTDouble(limelightName, "getpipe");
    }

    /**
     * Gets the current pipeline type.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline type string (e.g. "retro", "apriltag", etc)
     */
    public static String getCurrentPipelineType(String limelightName) {
        return getLimelightNTString(limelightName, "getpipetype");
    }

    /**
     * Gets the full JSON results dump.
     * @param limelightName Name of the Limelight camera
     * @return JSON string containing all current results
     */
    public static String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }

    /**
     * Switch to getBotPose
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    public static double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    public static double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    public static double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    /**
     * Gets the average color under the crosshair region as a 3-element array.
     * @param limelightName Name of the Limelight camera
     * @return Array containing [Blue, Green, Red] color values (BGR order)
     */
    public static double[] getTargetColor(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    /**
     * Gets the Limelight heartbeat value. Increments once per frame, allowing you to detect if the Limelight is connected and alive.
     * @param limelightName Name of the Limelight camera
     * @return Heartbeat value that increments each frame
     */
    public static double getHeartbeat(String limelightName) {
        return getLimelightNTDouble(limelightName, "hb");
    }

    public static String getNeuralClassID(String limelightName) {
        return getLimelightNTString(limelightName, "tclass");
    }

    public static String[] getRawBarcodeData(String limelightName) {
        return getLimelightNTStringArray(limelightName, "rawbarcodes");
    }

    /////
    /////

    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
        return toPose3D(poseArray);
    }

    /**
     * (Not Recommended) Gets the robot's 3D pose in the WPILib Red Alliance Coordinate System.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation in Red Alliance field space
     */
    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    /**
     * (Recommended) Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation in Blue Alliance field space
     */
    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    /**
     * Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation relative to the target
     */
    public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the camera's position and orientation relative to the target
     */
    public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the target's 3D pose with respect to the camera's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the camera
     */
    public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the target's 3D pose with respect to the robot's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the robot
     */
    public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the camera's 3D pose with respect to the robot's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the camera's position and orientation relative to the robot
     */
    public static Pose3d getCameraPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {

        double[] result = getBotPose_wpiBlue(limelightName);
        return toPose2D(result);
    }

    /**
     * Gets the MegaTag1 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * 
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * Make sure you are calling setRobotOrientation() before calling this method.
     * 
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {

        double[] result = getBotPose_wpiRed(limelightName);
        return toPose2D(result);

    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the RED
     * alliance
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired", false);
    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the RED
     * alliance
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d(String limelightName) {

        double[] result = getBotPose(limelightName);
        return toPose2D(result);

    }
   
    /**
     * Gets the current IMU data from NetworkTables.
     * IMU data is formatted as [robotYaw, Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ].
     * Returns all zeros if data is invalid or unavailable.
     * 
     * @param limelightName Name/identifier of the Limelight
     * @return IMUData object containing all current IMU data
     */
    public static IMUData getIMUData(String limelightName) {
        double[] imuData = getLimelightNTDoubleArray(limelightName, "imu");
        if (imuData == null || imuData.length < 10) {
            return new IMUData();  // Returns object with all zeros
        }
        return new IMUData(imuData);
    }

    /////
    /////

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    
    public static void setPriorityTagID(String limelightName, int ID) {
        setLimelightNTDouble(limelightName, "priorityid", ID);
    }

    /**
     * Sets LED mode to be controlled by the current pipeline.
     * @param limelightName Name of the Limelight camera
     */
    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    /**
     * Enables standard side-by-side stream mode.
     * @param limelightName Name of the Limelight camera
     */
    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    /**
     * Enables Picture-in-Picture mode with secondary stream in the corner.
     * @param limelightName Name of the Limelight camera
     */
    public static void setStreamMode_PiPMain(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    /**
     * Enables Picture-in-Picture mode with primary stream in the corner.
     * @param limelightName Name of the Limelight camera
     */
    public static void setStreamMode_PiPSecondary(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 2);
    }


    /**
     * Sets the crop window for the camera. The crop window in the UI must be completely open.
     * @param limelightName Name of the Limelight camera
     * @param cropXMin Minimum X value (-1 to 1)
     * @param cropXMax Maximum X value (-1 to 1)
     * @param cropYMin Minimum Y value (-1 to 1)
     * @param cropYMax Maximum Y value (-1 to 1)
     */
    public static void setCropWindow(String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropXMin;
        entries[1] = cropXMax;
        entries[2] = cropYMin;
        entries[3] = cropYMax;
        setLimelightNTDoubleArray(limelightName, "crop", entries);
    }

    /**
     * Sets the keystone modification for the crop window.
     * @param limelightName Name of the Limelight camera
     * @param horizontal Horizontal keystone value (-0.95 to 0.95)
     * @param vertical Vertical keystone value (-0.95 to 0.95)
     */
    public static void setKeystone(String limelightName, double horizontal, double vertical) {
        double[] entries = new double[2];
        entries[0] = horizontal;
        entries[1] = vertical;
        setLimelightNTDoubleArray(limelightName, "keystone_set", entries);
    }

    /**
     * Sets 3D offset point for easy 3D targeting.
     */
    public static void setFiducial3DOffset(String limelightName, double offsetX, double offsetY, double offsetZ) {
        double[] entries = new double[3];
        entries[0] = offsetX;
        entries[1] = offsetY;
        entries[2] = offsetZ;
        setLimelightNTDoubleArray(limelightName, "fiducial_offset_set", entries);
    }

    /**
     * Sets robot orientation values used by MegaTag2 localization algorithm.
     * 
     * @param limelightName Name/identifier of the Limelight
     * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
     * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
     * @param pitch (Unnecessary) Robot pitch in degrees 
     * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
     * @param roll (Unnecessary) Robot roll in degrees
     * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
     */
    public static void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
        double pitch, double pitchRate, 
        double roll, double rollRate) {
        SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
    }

    public static void SetRobotOrientation_NoFlush(String limelightName, double yaw, double yawRate, 
        double pitch, double pitchRate, 
        double roll, double rollRate) {
        SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, false);
    }

    private static void SetRobotOrientation_INTERNAL(String limelightName, double yaw, double yawRate, 
        double pitch, double pitchRate, 
        double roll, double rollRate, boolean flush) {

        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = yawRate;
        entries[2] = pitch;
        entries[3] = pitchRate;
        entries[4] = roll;
        entries[5] = rollRate;
        setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries);
        if(flush)
        {
            Flush();
        }
    }
   
    /**
     * Configures the IMU mode for MegaTag2 Localization
     * 
     * @param limelightName Name/identifier of the Limelight
     * @param mode IMU mode.
     */
    public static void SetIMUMode(String limelightName, int mode) {
        setLimelightNTDouble(limelightName, "imumode_set", mode);
    }

    /**
     * Configures the complementary filter alpha value for IMU Assist Modes (Modes 3 and 4)
     * 
     * @param limelightName Name/identifier of the Limelight
     * @param alpha Defaults to .001. Higher values will cause the internal IMU to converge onto the assist source more rapidly.
     */
    public static void SetIMUAssistAlpha(String limelightName, double alpha) {
        setLimelightNTDouble(limelightName, "imuassistalpha_set", alpha);
    }

    
    /**
     * Configures the throttle value. Set to 100-200 while disabled to reduce thermal output/temperature.
     * 
     * @param limelightName Name/identifier of the Limelight
     * @param throttle Defaults to 0. Your Limelgiht will process one frame after skipping &lt;throttle&gt; frames.
     */
    public static void SetThrottle(String limelightName, int throttle) {
        setLimelightNTDouble(limelightName, "throttle_set", throttle);
    }

    /**
     * Overrides the valid AprilTag IDs that will be used for localization.
     * Tags not in this list will be ignored for robot pose estimation.
     *
     * @param limelightName Name/identifier of the Limelight
     * @param validIDs Array of valid AprilTag IDs to track
     */
    public static void SetFiducialIDFiltersOverride(String limelightName, int[] validIDs) {
        double[] validIDsDouble = new double[validIDs.length];
        for (int i = 0; i < validIDs.length; i++) {
            validIDsDouble[i] = validIDs[i];
        }        
        setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
    }

    /**
     * Sets the downscaling factor for AprilTag detection.
     * Increasing downscale can improve performance at the cost of potentially reduced detection range.
     * 
     * @param limelightName Name/identifier of the Limelight
     * @param downscale Downscale factor. Valid values: 1.0 (no downscale), 1.5, 2.0, 3.0, 4.0. Set to 0 for pipeline control.
     */
    public static void SetFiducialDownscalingOverride(String limelightName, float downscale) 
    {
        int d = 0; // pipeline
        if (downscale == 1.0)
        {
            d = 1;
        }
        if (downscale == 1.5)
        {
            d = 2;
        }
        if (downscale == 2)
        {
            d = 3;
        }
        if (downscale == 3)
        {
            d = 4;
        }
        if (downscale == 4)
        {
            d = 5;
        }
        setLimelightNTDouble(limelightName, "fiducial_downscale_set", d);
    }
    
    /**
     * Sets the camera pose relative to the robot.
     * @param limelightName Name of the Limelight camera
     * @param forward Forward offset in meters
     * @param side Side offset in meters
     * @param up Up offset in meters
     * @param roll Roll angle in degrees
     * @param pitch Pitch angle in degrees
     * @param yaw Yaw angle in degrees
     */
    public static void setCameraPose_RobotSpace(String limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    /////
    /////

    public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
        setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
    }

    public static double[] getPythonScriptData(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }

    /////
    /////

    /**
     * Triggers a snapshot capture via NetworkTables by incrementing the snapshot counter.
     * Rate-limited to once per 10 frames on the Limelight.
     * @param limelightName Name of the Limelight camera
     */
    public static void triggerSnapshot(String limelightName) {
        double current = getLimelightNTDouble(limelightName, "snapshot");
        setLimelightNTDouble(limelightName, "snapshot", current + 1);
    }

    /**
     * Enables or pauses the rewind buffer recording.
     * @param limelightName Name of the Limelight camera
     * @param enabled True to enable recording, false to pause
     */
    public static void setRewindEnabled(String limelightName, boolean enabled) {
        setLimelightNTDouble(limelightName, "rewind_enable_set", enabled ? 1 : 0);
    }

    /**
     * Triggers a rewind capture with the specified duration.
     * Maximum duration is 165 seconds. Rate-limited on the Limelight.
     * @param limelightName Name of the Limelight camera
     * @param durationSeconds Duration of rewind capture in seconds (max 165)
     */
    public static void triggerRewindCapture(String limelightName, double durationSeconds) {
        double[] currentArray = getLimelightNTDoubleArray(limelightName, "capture_rewind");
        double counter = (currentArray.length > 0) ? currentArray[0] : 0;
        double[] entries = new double[2];
        entries[0] = counter + 1;
        entries[1] = Math.min(durationSeconds, 165);
        setLimelightNTDoubleArray(limelightName, "capture_rewind", entries);
    }

    /**
     * Gets the latest JSON results output and returns a LimelightResults object.
     * @param limelightName Name of the Limelight camera
     * @return LimelightResults object containing all current target data
     */
    public static LimelightResults getLatestResults(String limelightName) {

        long start = System.nanoTime();
        LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            String jsonString = getJSONDump(limelightName);
            if (jsonString == null || jsonString.isEmpty() || jsonString.isBlank()) {
                results.error = "lljson error: empty json";
            } else {
                results = mapper.readValue(jsonString, LimelightResults.class);
                if (results.imuResults != null) {
                    results.imuResults.parseDataArray();
                }
            }
        } catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.latency_jsonParse = millis;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return results;
    }

    /**
     * Sets up port forwarding for a Limelight 3A/3G connected via USB.
     * This allows access to the Limelight web interface and video stream
     * when connected to the robot over USB.
     *
     * For usbIndex 0: ports 5800-5809 forward to 172.29.0.1
     * For usbIndex 1: ports 5810-5819 forward to 172.29.1.1
     * etc.
     *
     * Call this method once during robot initialization.
     * To access the interface of the camera with usbIndex0, you would go to roboRIO-(teamnum)-FRC.local:5801. Port 5811 for usb index 1
     *
     * @param usbIndex The USB index of the Limelight (0, 1, 2, etc.)
     */
    public static void setupPortForwardingUSB(int usbIndex) {
        String ip = "172.29." + usbIndex + ".1";
        int basePort = 5800 + (usbIndex * 10);

        for (int i = 0; i < 10; i++) {
            PortForwarder.add(basePort + i, ip, 5800 + i);
        }
    }
}
