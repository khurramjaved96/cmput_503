
"use strict";

let Vsample = require('./Vsample.js');
let EncoderStamped = require('./EncoderStamped.js');
let LineFollowerStamped = require('./LineFollowerStamped.js');
let TagInfo = require('./TagInfo.js');
let TurnIDandType = require('./TurnIDandType.js');
let CarControl = require('./CarControl.js');
let CoordinationClearance = require('./CoordinationClearance.js');
let DiagnosticsCodeProfiling = require('./DiagnosticsCodeProfiling.js');
let VehicleCorners = require('./VehicleCorners.js');
let StreetNameDetection = require('./StreetNameDetection.js');
let SourceTargetNodes = require('./SourceTargetNodes.js');
let IntersectionPoseImgDebug = require('./IntersectionPoseImgDebug.js');
let DisplayFragment = require('./DisplayFragment.js');
let EpisodeStart = require('./EpisodeStart.js');
let DroneControl = require('./DroneControl.js');
let MaintenanceState = require('./MaintenanceState.js');
let LightSensor = require('./LightSensor.js');
let AprilTagsWithInfos = require('./AprilTagsWithInfos.js');
let BoolStamped = require('./BoolStamped.js');
let LEDInterpreter = require('./LEDInterpreter.js');
let ObstacleImageDetectionList = require('./ObstacleImageDetectionList.js');
let LanePose = require('./LanePose.js');
let WheelsCmd = require('./WheelsCmd.js');
let AprilTagDetectionArray = require('./AprilTagDetectionArray.js');
let KinematicsParameters = require('./KinematicsParameters.js');
let StreetNames = require('./StreetNames.js');
let DuckieSensor = require('./DuckieSensor.js');
let Pose2DStamped = require('./Pose2DStamped.js');
let DiagnosticsRosNode = require('./DiagnosticsRosNode.js');
let DiagnosticsRosProfilingUnit = require('./DiagnosticsRosProfilingUnit.js');
let WheelsCmdStamped = require('./WheelsCmdStamped.js');
let Vector2D = require('./Vector2D.js');
let ObstacleProjectedDetection = require('./ObstacleProjectedDetection.js');
let SegmentList = require('./SegmentList.js');
let Trajectory = require('./Trajectory.js');
let SignalsDetectionETHZ17 = require('./SignalsDetectionETHZ17.js');
let LEDDetectionArray = require('./LEDDetectionArray.js');
let DiagnosticsRosTopicArray = require('./DiagnosticsRosTopicArray.js');
let LEDDetection = require('./LEDDetection.js');
let SignalsDetection = require('./SignalsDetection.js');
let DiagnosticsRosParameterArray = require('./DiagnosticsRosParameterArray.js');
let WheelEncoderStamped = require('./WheelEncoderStamped.js');
let ButtonEvent = require('./ButtonEvent.js');
let AprilTagDetection = require('./AprilTagDetection.js');
let NodeParameter = require('./NodeParameter.js');
let Pixel = require('./Pixel.js');
let DiagnosticsRosProfiling = require('./DiagnosticsRosProfiling.js');
let CoordinationSignal = require('./CoordinationSignal.js');
let DiagnosticsCodeProfilingArray = require('./DiagnosticsCodeProfilingArray.js');
let DroneMode = require('./DroneMode.js');
let Twist2DStamped = require('./Twist2DStamped.js');
let VehiclePose = require('./VehiclePose.js');
let DiagnosticsRosLinkArray = require('./DiagnosticsRosLinkArray.js');
let ThetaDotSample = require('./ThetaDotSample.js');
let ParamTuner = require('./ParamTuner.js');
let AntiInstagramThresholds = require('./AntiInstagramThresholds.js');
let StopLineReading = require('./StopLineReading.js');
let IntersectionPoseImg = require('./IntersectionPoseImg.js');
let ObstacleProjectedDetectionList = require('./ObstacleProjectedDetectionList.js');
let IntersectionPose = require('./IntersectionPose.js');
let ObstacleImageDetection = require('./ObstacleImageDetection.js');
let Segment = require('./Segment.js');
let DiagnosticsRosTopic = require('./DiagnosticsRosTopic.js');
let ObstacleType = require('./ObstacleType.js');
let KinematicsWeights = require('./KinematicsWeights.js');
let SceneSegments = require('./SceneSegments.js');
let Rect = require('./Rect.js');
let DiagnosticsRosLink = require('./DiagnosticsRosLink.js');
let Rects = require('./Rects.js');
let LEDDetectionDebugInfo = require('./LEDDetectionDebugInfo.js');
let WheelsCmdDBV2Stamped = require('./WheelsCmdDBV2Stamped.js');
let FSMState = require('./FSMState.js');
let DuckiebotLED = require('./DuckiebotLED.js');
let LEDPattern = require('./LEDPattern.js');

module.exports = {
  Vsample: Vsample,
  EncoderStamped: EncoderStamped,
  LineFollowerStamped: LineFollowerStamped,
  TagInfo: TagInfo,
  TurnIDandType: TurnIDandType,
  CarControl: CarControl,
  CoordinationClearance: CoordinationClearance,
  DiagnosticsCodeProfiling: DiagnosticsCodeProfiling,
  VehicleCorners: VehicleCorners,
  StreetNameDetection: StreetNameDetection,
  SourceTargetNodes: SourceTargetNodes,
  IntersectionPoseImgDebug: IntersectionPoseImgDebug,
  DisplayFragment: DisplayFragment,
  EpisodeStart: EpisodeStart,
  DroneControl: DroneControl,
  MaintenanceState: MaintenanceState,
  LightSensor: LightSensor,
  AprilTagsWithInfos: AprilTagsWithInfos,
  BoolStamped: BoolStamped,
  LEDInterpreter: LEDInterpreter,
  ObstacleImageDetectionList: ObstacleImageDetectionList,
  LanePose: LanePose,
  WheelsCmd: WheelsCmd,
  AprilTagDetectionArray: AprilTagDetectionArray,
  KinematicsParameters: KinematicsParameters,
  StreetNames: StreetNames,
  DuckieSensor: DuckieSensor,
  Pose2DStamped: Pose2DStamped,
  DiagnosticsRosNode: DiagnosticsRosNode,
  DiagnosticsRosProfilingUnit: DiagnosticsRosProfilingUnit,
  WheelsCmdStamped: WheelsCmdStamped,
  Vector2D: Vector2D,
  ObstacleProjectedDetection: ObstacleProjectedDetection,
  SegmentList: SegmentList,
  Trajectory: Trajectory,
  SignalsDetectionETHZ17: SignalsDetectionETHZ17,
  LEDDetectionArray: LEDDetectionArray,
  DiagnosticsRosTopicArray: DiagnosticsRosTopicArray,
  LEDDetection: LEDDetection,
  SignalsDetection: SignalsDetection,
  DiagnosticsRosParameterArray: DiagnosticsRosParameterArray,
  WheelEncoderStamped: WheelEncoderStamped,
  ButtonEvent: ButtonEvent,
  AprilTagDetection: AprilTagDetection,
  NodeParameter: NodeParameter,
  Pixel: Pixel,
  DiagnosticsRosProfiling: DiagnosticsRosProfiling,
  CoordinationSignal: CoordinationSignal,
  DiagnosticsCodeProfilingArray: DiagnosticsCodeProfilingArray,
  DroneMode: DroneMode,
  Twist2DStamped: Twist2DStamped,
  VehiclePose: VehiclePose,
  DiagnosticsRosLinkArray: DiagnosticsRosLinkArray,
  ThetaDotSample: ThetaDotSample,
  ParamTuner: ParamTuner,
  AntiInstagramThresholds: AntiInstagramThresholds,
  StopLineReading: StopLineReading,
  IntersectionPoseImg: IntersectionPoseImg,
  ObstacleProjectedDetectionList: ObstacleProjectedDetectionList,
  IntersectionPose: IntersectionPose,
  ObstacleImageDetection: ObstacleImageDetection,
  Segment: Segment,
  DiagnosticsRosTopic: DiagnosticsRosTopic,
  ObstacleType: ObstacleType,
  KinematicsWeights: KinematicsWeights,
  SceneSegments: SceneSegments,
  Rect: Rect,
  DiagnosticsRosLink: DiagnosticsRosLink,
  Rects: Rects,
  LEDDetectionDebugInfo: LEDDetectionDebugInfo,
  WheelsCmdDBV2Stamped: WheelsCmdDBV2Stamped,
  FSMState: FSMState,
  DuckiebotLED: DuckiebotLED,
  LEDPattern: LEDPattern,
};
