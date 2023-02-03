# Limelightlib (WPILIB Java)

## Usage

This is a single-file library. All you need to do is copy the file into your Java project's "robot" folder. You don't need to create any objects for your Limelights - the library is designed to be used in a functional manner.

### Basic Usage
Every method in Limelightlib accepts a string parameter indicating the correct Limelight to use. If left blank or null, the name is assumed to be "limelight"
```
LimelightHelpers.setLEDMode_PipelineControl("");
LimelightHelpers.setLEDMode_ForceBlink("")
LimelightHelpers.setCropWindow("",-1,1,-1,1);
double tx = LimelightHelpers.getTX("");
```

### JSON Parsing
getLatestResults() parses the latest Limelight JSON dump into a LimelightResults object.
```
LimelightResults llresults = LimelightHelpers.getLatestResults("");
```
Each LimelightResults instance contains a Results object. Each Results object contains data such as botpose, an array for each target type, etc. With getLatestResults(), you now have easy access to 100% of your Limelight's output.

```
double[] botposeRed = llresults.results.botpose_wpired;
double pipelineLatency = llresults.results.latency_pipeline;
LimelightTarget_Fiducial = llresults.results.targets_Fiducials;
```

### Taking snapshots
```
LimelightHelpers.takeSnapshot("","snapshotname");
```