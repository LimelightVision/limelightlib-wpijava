import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightObjectDetection {

    public static boolean isTargetDetected() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    }

    public static double getTargetX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public static double getTargetY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    public static double getTargetArea() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }

    public static void setPipeline(int pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }

    public static void main(String[] args) {
        if (isTargetDetected()) {
            System.out.println("Target detected!");
            System.out.println("X: " + getTargetX());
            System.out.println("Y: " + getTargetY());
            System.out.println("Area: " + getTargetArea());
        } else {
            System.out.println("No target detected.");
        }
    }
}