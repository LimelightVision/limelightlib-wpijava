# Limelightlib (WPILIB Java)
https://github.com/LimelightVision/limelightlib-wpijava/releases
## Usage

This is a single-file library. All you need to do is copy the LimelightHelpers.java file from the latest release (https://github.com/LimelightVision/limelightlib-wpijava/releases) into your Java project's "robot" folder. You don't need to create any objects for your Limelights - the library is designed to be used in a functional manner.

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
Takes up to 2.5ms on RoboRIO 1.0. Parsing latency is logged in results.targetingresults.latency_jsonParse and may be added to latency_pipeline and latency_capture.

```
LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
```
Each LimelightResults instance contains a Results object. Each Results object contains data such as botpose, an array for each target type, etc. With getLatestResults(), you now have easy access to 100% of your Limelight's output.

```
double[] botposeRed = llresults.results.botpose_wpired;
double pipelineLatency = llresults.results.latency_pipeline;
LimelightHelpers.LimelightTarget_Fiducial = llresults.results.targets_Fiducials;
```

### Taking snapshots
```
LimelightHelpers.takeSnapshot("","snapshotname");
```

### Classes
```
LimelightHelpers.LimelightTarget_Retro
LimelightHelpers.LimelightTarget_Fiducial
LimelightHelpers.LimelightTarget_Barcode
LimelightHelpers.LimelightTarget_Classifier
LimelightHelpers.LimelightTarget_Detector
LimelightHelpers.Results
LimelightHelpers.LimelightResults
(Pure Static) LimelightHelpers
```

### LimelightHelpers Methods
```
getLimelightNTTable()
getLimelightNTTableEntry()
getLimelightNTDouble()
setLimelightNTDouble()
setLimelightNTDoubleArray()
getLimelightNTDouleArray()
getLimelightNTString()
getLimelightURLString()

getTX()
getTY()
getTA()
getLatency_Pipeline()
getLatency_Capture()
getCurrentPipelineIndex()
getJsonDump()

getBotpose()
getBotpose_wpiRed()
getBotpose_wpiBlue()
getBotpose_TargetSpace
getCameraPsoe_TargetSpace()
getTargetPose_CameraSpace()
getTargetPose_RobotSpace()

getTargetColor()
getFiducialID()
getNeuralClassID()

setPipelineIndex()

setLEDMode_PipelineControl()
setLEDMode_ForceOff()
setLEDMode_ForceBlink()
setLEDMode_ForceOn()

setStreamMode_Standard()
setStreamMode_PiPMain()
setStreamMode_PiPSecondary()

setCropWindow()

setPythonScriptData()
getPythonScriptData()

takeSnapshot()
getLatestResults()
```
