
"use strict";

let GetControllerDigitalIO = require('./GetControllerDigitalIO.js')
let GripperState = require('./GripperState.js')
let SetLoad = require('./SetLoad.js')
let GetAnalogIO = require('./GetAnalogIO.js')
let GetErr = require('./GetErr.js')
let ConfigToolModbus = require('./ConfigToolModbus.js')
let SetMultipleInts = require('./SetMultipleInts.js')
let TCPOffset = require('./TCPOffset.js')
let SetString = require('./SetString.js')
let PlayTraj = require('./PlayTraj.js')
let SetControllerAnalogIO = require('./SetControllerAnalogIO.js')
let ClearErr = require('./ClearErr.js')
let SetFloat32 = require('./SetFloat32.js')
let MoveVelo = require('./MoveVelo.js')
let Move = require('./Move.js')
let FtCaliLoad = require('./FtCaliLoad.js')
let SetToolModbus = require('./SetToolModbus.js')
let MoveVelocity = require('./MoveVelocity.js')
let GetDigitalIO = require('./GetDigitalIO.js')
let SetModbusTimeout = require('./SetModbusTimeout.js')
let Call = require('./Call.js')
let GripperConfig = require('./GripperConfig.js')
let MoveAxisAngle = require('./MoveAxisAngle.js')
let GripperMove = require('./GripperMove.js')
let SetAxis = require('./SetAxis.js')
let GetInt32 = require('./GetInt32.js')
let SetDigitalIO = require('./SetDigitalIO.js')
let GetSetModbusData = require('./GetSetModbusData.js')
let FtIdenLoad = require('./FtIdenLoad.js')
let GetFloat32List = require('./GetFloat32List.js')
let SetInt16 = require('./SetInt16.js')

module.exports = {
  GetControllerDigitalIO: GetControllerDigitalIO,
  GripperState: GripperState,
  SetLoad: SetLoad,
  GetAnalogIO: GetAnalogIO,
  GetErr: GetErr,
  ConfigToolModbus: ConfigToolModbus,
  SetMultipleInts: SetMultipleInts,
  TCPOffset: TCPOffset,
  SetString: SetString,
  PlayTraj: PlayTraj,
  SetControllerAnalogIO: SetControllerAnalogIO,
  ClearErr: ClearErr,
  SetFloat32: SetFloat32,
  MoveVelo: MoveVelo,
  Move: Move,
  FtCaliLoad: FtCaliLoad,
  SetToolModbus: SetToolModbus,
  MoveVelocity: MoveVelocity,
  GetDigitalIO: GetDigitalIO,
  SetModbusTimeout: SetModbusTimeout,
  Call: Call,
  GripperConfig: GripperConfig,
  MoveAxisAngle: MoveAxisAngle,
  GripperMove: GripperMove,
  SetAxis: SetAxis,
  GetInt32: GetInt32,
  SetDigitalIO: SetDigitalIO,
  GetSetModbusData: GetSetModbusData,
  FtIdenLoad: FtIdenLoad,
  GetFloat32List: GetFloat32List,
  SetInt16: SetInt16,
};
