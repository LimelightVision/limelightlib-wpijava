// LimelightHelpers v1.11 (REQUIRES LLOS 2025.0 OR LATER)

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

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.Map;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.concurrent.ConcurrentHashMap;

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

        /**
         * The Limelight's position in the target's space (its location and angle relative to the target,
         * meaning that the target is the origin).
         */
        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        /**
         * The robot's position in the overall field (its location and angle in field coordinates).
         */
        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        /**
         * The robot's position in the target's space (its location and angle relative to the target,
         * meaning that the target is the origin). It uses the Limelight's configured position to calculate this.
         */
        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        /**
         * The target's position in the Limelight's view (its location and angle relative to the Limelight).
         * It's just the inverse of cameraPose_TargetSpace.
         */
        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        /**
         * The target's position in the robot's space (its location and angle relative to the robot,
         * meaning that the robot is the origin).
         */
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

        /**
         * The area of the target in the Limelight's view as a percentage of the total image area.
         * So, if the target takes up 10% of the image, this value will be 0.1.
         */
        @JsonProperty("ta")
        public double ta;

        /**
         * The horizontal offset of the target from the Limelight's crosshair in degrees. Positive values 
         * mean the target is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal fov.
         */
        @JsonProperty("tx")
        public double tx;
        
        /**
         * The vertical offset of the target from the Limelight's crosshair in degrees. Positive values 
         * mean the target is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical fov.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The horizontal offset of the target from the Limelight's crosshair in pixels. Positive values 
         * mean the target is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal resolution.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The vertical offset of the target from the Limelight's crosshair in pixels. Positive values 
         * mean the target is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical resolution.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        /**
         * The horizontal offset of the target from the Limelight's principle pixel in degrees. The principle
         * pixel is the center of the image, so if your crosshair is aligned with the principle pixel, this value
         * will be the same as tx.
         */
        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        /**
         * The vertical offset of the target from the Limelight's principle pixel in degrees. The principle
         * pixel is the center of the image, so if your crosshair is aligned with the principle pixel, this value
         * will be the same as ty.
         */
        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        /**
         * The timestamp of the target data in milliseconds since the Limelight booted up.
         */
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

        /**
         * The ID of the fiducial target (april tag). This is a unique identifier for the target.
         * For the 2025 FRC season they will range from 1-22.
         */
        @JsonProperty("fID")
        public double fiducialID;

        /**
         * The family of the fiducial target. The 36h11 tag family is used for the 2025 FRC season.
         */
        @JsonProperty("fam")
        public String fiducialFamily;

        /**
         * The Limelight's position in the target's space (its location and angle relative to the target,
         * meaning that the april tag is the origin).
         */
        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        /**
         * The robot's position in the overall field (its location and angle in field coordinates).
         */
        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        /**
         * The robot's position in the target's space (its location and angle relative to the target,
         * meaning that the april tag is the origin).
         * It uses the Limelight's configured position to calculate this.
         */
        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        /**
         * The target's position in the Limelight's view (its location and angle relative to the Limelight).
         * It's just the inverse of cameraPose_TargetSpace.
         */
        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        /**
         * The target's position in the robot's space (its location and angle relative to the robot,
         * meaning that the robot is the origin).
         */
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
        
        /**
         * The area of the april tag in the Limelight's view as a percentage of the total image area.
         * So, if the april tag takes up 10% of the image, this value will be 0.1.
         */
        @JsonProperty("ta")
        public double ta;

        /**
         * The horizontal offset of the april tag from the Limelight's crosshair in degrees. Positive values 
         * mean the april tag is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal fov.
         */
        @JsonProperty("tx")
        public double tx;

        /**
         * The vertical offset of the april tag from the Limelight's crosshair in degrees. Positive values 
         * mean the april tag is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical fov.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The horizontal offset of the april tag from the Limelight's crosshair in pixels. Positive values 
         * mean the april tag is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal resolution.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The vertical offset of the april tag from the Limelight's crosshair in pixels. Positive values 
         * mean the april tag is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical resolution.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        /**
         * The horizontal offset of the april tag from the Limelight's principle pixel in degrees. The principle
         * pixel is the center of the image, so if your crosshair is aligned with the principle pixel, this value
         * will be the same as tx.
         */
        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        /**
         * The vertical offset of the april tag from the Limelight's principle pixel in degrees. The principle
         * pixel is the center of the image, so if your crosshair is aligned with the principle pixel, this value
         * will be the same as ty.
         */
        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        /**
         * The timestamp of the april tag data in milliseconds since the Limelight booted up.
         */
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

        /**
         * The name of the classified object or category.
         */
        @JsonProperty("class")
        public String className;

        /**
         * The numeric identifier of the classified object or category.
         */
        @JsonProperty("classID")
        public double classID;

        /**
         * The confidence score of the classification, ranging from 0 to 1.
         */
        @JsonProperty("conf")
        public double confidence;

        /**
         * The zone or area ID where the classified object is located (user-defined regions).
         */
        @JsonProperty("zone")
        public double zone;

        /**
         * The horizontal offset of the classified object from the Limelight's crosshair in degrees. Positive values 
         * mean the object is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal fov.
         */
        @JsonProperty("tx")
        public double tx;

        /**
         * The horizontal offset of the classified object from the Limelight's crosshair in pixels. Positive values 
         * mean the object is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal resolution.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The vertical offset of the classified object from the Limelight's crosshair in degrees. Positive values 
         * mean the object is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical fov.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The vertical offset of the classified object from the Limelight's crosshair in pixels. Positive values 
         * mean the object is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical resolution.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTarget_Classifier() {}
    }

    /**
     * Represents a Neural Detector Pipeline Result extracted from JSON Output
     */ 
    public static class LimelightTarget_Detector {

        /**
         * The name of the detected object or category.
         */
        @JsonProperty("class")
        public String className;

        /**
         * The numeric identifier of the detected object or category.
         */
        @JsonProperty("classID")
        public double classID;

        /**
         * The confidence score of the detection, ranging from 0 to 1.
         */
        @JsonProperty("conf")
        public double confidence;

        /**
         * The area of the detected object in the Limelight's view as a percentage of the total image area.
         * So, if the object takes up 10% of the image, this value will be 0.1. This is very helpful for
         * estimating distance to the object based on its size.
         */
        @JsonProperty("ta")
        public double ta;

        /**
         * The horizontal offset of the detected object from the Limelight's crosshair in degrees. Positive values 
         * mean the object is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal fov.
         */
        @JsonProperty("tx")
        public double tx;

        /**
         * The vertical offset of the detected object from the Limelight's crosshair in degrees. Positive values 
         * mean the object is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical fov.
         */
        @JsonProperty("ty")
        public double ty;

        /**
         * The horizontal offset of the detected object from the Limelight's crosshair in pixels. Positive values 
         * mean the object is to the right of the crosshair. The bounds of this value depend on the Limelight's
         * horizontal resolution.
         */
        @JsonProperty("txp")
        public double tx_pixels;

        /**
         * The vertical offset of the detected object from the Limelight's crosshair in pixels. Positive values 
         * mean the object is below the crosshair. The bounds of this value depend on the Limelight's
         * vertical resolution.
         */
        @JsonProperty("typ")
        public double ty_pixels;

        /**
         * The horizontal offset of the detected object from the Limelight's principle pixel in degrees. The principle
         * pixel is the center of the image, so if your crosshair is aligned with the principle pixel, this value
         * will be the same as tx.
         */
        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        /**
         * The vertical offset of the detected object from the Limelight's principle pixel in degrees. The principle
         * pixel is the center of the image, so if your crosshair is aligned with the principle pixel, this value
         * will be the same as ty.
         */
        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        public LimelightTarget_Detector() {}
    }

    /**
     * Limelight Results object, parsed from a Limelight's JSON results output.
     */
    public static class LimelightResults {
        
        /**
         * The error message (if any) associated with getting the LimelightResults
         */
        public String error;
        
        /**
         * The pipeline ID that the Limelight is currently using.
         */
        @JsonProperty("pID")
        public double pipelineID;

        /**
         * The latency of the Limelight's image processing pipeline in milliseconds.
         */
        @JsonProperty("tl")
        public double latency_pipeline;

        /**
         * The latency of the Limelight's image capture in milliseconds.
         */
        @JsonProperty("cl")
        public double latency_capture;

        /**
         * The latency of how long JSON parsing took in milliseconds.
         */
        public double latency_jsonParse;

        /**
         * The timestamp of the Limelight's data in milliseconds since the Limelight booted up.
         */
        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        /**
         * The timestamp of the image capture event, synchronized to the RoboRIO's FPGA clock (seconds).
         * Uses the same time base as `Timer.getFPGATimestamp()`.
         */
        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        /**
         * The validity indicator. 1 = valid targets, 0 = no valid targets
         */
        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        /**
         * The MegaTag1 pose estimate from the Limelight. It's in an array of 6 values: 
         * [x, y, z, roll, pitch, yaw] (meters, degrees).
         */
        @JsonProperty("botpose")
        public double[] botpose;

        /**
         * The MegaTag1 pose estimate from the Limelight from the red alliance's perspective. The
         * origin is the bottom right corner of the field. It's in an array of 6 values: [x, y, z, 
         * roll, pitch, yaw] (meters, degrees).
         */
        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        /**
         * The MegaTag1 pose estimate from the Limelight from the blue alliance's perspective. The
         * origin is the bottom left corner of the field. It's in an array of 6 values: [x, y, z, 
         * roll, pitch, yaw] (meters, degrees).
         */
        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        /**
         * The number of april tags used to calculate the MegaTag1 pose estimate.
         */
        @JsonProperty("botpose_tagcount")
        public double botpose_tagcount;
       
        /**
         * The max distance, in meters, between april tags used to calculate the MegaTag1 pose estimate.
         */
        @JsonProperty("botpose_span")
        public double botpose_span;
       
        /**
         * The average distance, in meters, between the Limelight and the april tags used to calculate 
         * the MegaTag1 pose estimate.
         */
        @JsonProperty("botpose_avgdist")
        public double botpose_avgdist;
       
        /**
         * The average area, in meters squared, of april tags used to calculate the MegaTag1 pose estimate.
         */
        @JsonProperty("botpose_avgarea")
        public double botpose_avgarea;

        /**
         * The Limelight's position in the robot's space (its location and angle relative to the robot,
         * meaning that the robot is the origin).
         */
        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

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

        /**
         * An array of Retroreflective Target results from the Limelight.
         */
        @JsonProperty("Retro")
        public LimelightTarget_Retro[] targets_Retro;

        /**
         * An array of Fiducial Target (april tag) results from the Limelight.
         */
        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        /**
         * An array of Classifier Target results from the Limelight.
         */
        @JsonProperty("Classifier")
        public LimelightTarget_Classifier[] targets_Classifier;

        /**
         * An array of Detector Target results from the Limelight.
         */
        @JsonProperty("Detector")
        public LimelightTarget_Detector[] targets_Detector;

        /**
         * An array of Barcode Target results from the Limelight.
         */
        @JsonProperty("Barcode")
        public LimelightTarget_Barcode[] targets_Barcode;

        public LimelightResults() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            targets_Classifier = new LimelightTarget_Classifier[0];
            targets_Detector = new LimelightTarget_Detector[0];
            targets_Barcode = new LimelightTarget_Barcode[0];
        }

    }

    /**
     * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
     */
    public static class RawFiducial {
        /** The id of the april tag */
        public int id = 0;
        /** 
         * The horizontal offset of the target from the Limelight's principle pixel (the images central pixel) in degrees. 
         * Positive values mean the target is to the right of the crosshair.
         */
        public double txnc = 0;
        /** 
         * The vertical offset of the target from the Limelight's principle pixel (the images central pixel) in degrees. 
         * Positive values mean the target is below the crosshair.
         */
        public double tync = 0;
        /** 
         * The area of the target in the Limelight's view as a percentage of the total image area.
         * So, if the target takes up 10% of the image, this value will be 0.1.
         */
        public double ta = 0;
        /** The distance from the Limelight to the april tag in meters. */
        public double distToCamera = 0;
        /** The distance from the robot to the april tag in meters. */
        public double distToRobot = 0;
        /** 
         * The ambiguity of the april tag. This ranges from 0 to 1, a lower values meaning less ambiguous. The
         * higher this is, the more likely it is that you will see ambiguity flipping. Ambiguity flipping is when
         * there are multiple possible "solutions" for the pose estimate, so if this is higher your estimate will
         * be less reliable. The best way to reduce this it to make the april tag look as trapezoidal as possible
         * from the Limelight's perspective. 
         */
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
        /** 
         * The estimated 2D pose of the robot, including position and rotation.
         */
        public Pose2d pose;
        /** 
         * The timestamp of the pose estimate in seconds since the Limelight booted up.
         */
        public double timestampSeconds;
        /** 
         * The latency of the pose estimate in milliseconds.
         */
        public double latency;
        /** 
         * The number of april tags used to calculate the pose estimate.
         */
        public int tagCount;
        /** 
         * The max distance, in meters, between april tags used to calculate the pose estimate.
         */
        public double tagSpan;
        /** 
         * The average distance, in meters, between the Limelight and the april tags used to calculate 
         * the pose estimate.
         */
        public double avgTagDist;
        /** 
         * The average area, in meters squared, of april tags used to calculate the pose estimate.
         */
        public double avgTagArea;
        /** 
         * An array of RawFiducial used to calculate the pose estimate.
         */
        public RawFiducial[] rawFiducials;
        /**
         * true if the pose estimate is calculated using MegaTag2, false if using MegaTag1.
         */
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
            return null; // or some default PoseEstimate
        }
    
        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int)extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);
        
        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
    
        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
    
        if (poseArray.length != expectedTotalVals) {
            // Don't populate fiducials
        } else {
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


    public static URL getLimelightURLString(String tableName, String request) {
        String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
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
        return (int)t2d[10];
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
        return (int)t2d[11];
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

    public static double[] getTargetColor(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
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
     * Sets the 3D point-of-interest offset for the current fiducial pipeline. 
     * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
     *
     * @param limelightName Name/identifier of the Limelight
     * @param x X offset in meters
     * @param y Y offset in meters
     * @param z Z offset in meters
     */
    public static void SetFidcuial3DOffset(String limelightName, double x, double y, 
        double z) {

        double[] entries = new double[3];
        entries[0] = x;
        entries[1] = y;
        entries[2] = z;
        setLimelightNTDoubleArray(limelightName, "fiducial_offset_set", entries);
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
     * Asynchronously take snapshot.
     */
    public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
        });
    }

    private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
        URL url = getLimelightURLString(tableName, "capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && !"".equals(snapshotName)) {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
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
            results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
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
}