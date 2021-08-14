-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- this file serves two purposes:
-- A) generic outgauge implementation: used to use for example custom made dashboard hardware and alike
-- B) update the user interface for the remote control app directly via the sendPackage function
-- please note that use case A and B exclude each other for now.

-- Beamstate Modifications

local next = next

local latches = {}

local function getLatches()
  local i = 1
  for _, b in pairs(v.data.beams) do
    if b.breakGroup ~= nil then
      local breakGroups = type(b.breakGroup) == "table" and b.breakGroup or {b.breakGroup}
      -- multiple break groups
      for _, g in pairs(breakGroups) do
        if type(g) == "string" and string.find(g, "latch") ~= nil then
          --log('D', "beamstate.breakHinges","  breaking hinge beam "..k.. " as in breakgroup ".. b.breakGroup)
          latches[i] = b
		  i = i + 1
          break
        end
      end
    end
  end
  return latches
end

local function getDoorsOpen()
  if next(latches) == nil then --Check if latches is populated
    getLatches()			   --Get latches if not
  end
  for _, b in pairs(latches) do
    if string.find(tostring(b.breakGroup), "door") ~= nil then
	  if obj:beamIsBroken(b.cid) then
	    return true
      end
	end
  end
  return false
end

local function getTrunkOpen()
  if next(latches) == nil then --Check if latches is populated
    getLatches()			   --Get latches if not
  end
  for _, b in pairs(latches) do
    if (string.find(tostring(b.breakGroup), "trunk") ~= nil or string.find(tostring(b.breakGroup), "hatch") ~= nil or string.find(tostring(b.breakGroup), "tailgate") ~= nil) then
	  if obj:beamIsBroken(b.cid) then
	    return true
	  end
	end
  end
  return false
end

-- END

-- Tprint

function tprint (tbl, indent)
  if not indent then indent = 0 end
  local toprint = string.rep(" ", indent) .. "{\r\n"
  indent = indent + 2 
  for k, v in pairs(tbl) do
    toprint = toprint .. string.rep(" ", indent)
    if (type(k) == "number") then
      toprint = toprint .. "[" .. k .. "] = "
    elseif (type(k) == "string") then
      toprint = toprint  .. k ..  "= "   
    end
    if (type(v) == "number") then
      toprint = toprint .. v .. ",\r\n"
    elseif (type(v) == "string") then
      toprint = toprint .. "\"" .. v .. "\",\r\n"
    elseif (type(v) == "table") then
      toprint = toprint .. tprint(v, indent + 2) .. ",\r\n"
    else
      toprint = toprint .. "\"" .. tostring(v) .. "\",\r\n"
    end
  end
  toprint = toprint .. string.rep(" ", indent-2) .. "}"
  return toprint
end

-- END

-- ESC

local function getESCMode()
  return currentESCConfiguration
end

local isEscEnabled = false
if controller.getController('yawControl') ~= nil then
	local comp = {}
	comp.type = "auxiliary"
	comp.defaultOrder = 70
	--comp.componentOrderTractionControl = 30
	comp.componentOrderYawControl = 40
	comp.isActive = false

	function comp.setParameters(parameters)
	  isEscEnabled = parameters["yawControl.isEnabled"]
	end

	function comp.actAsYawControl(measuredYaw, expectedYaw, yawDifference, bodySlipAngle, dt)
		return false
	end

	controller.getController('yawControl').registerComponent(comp)
end

-- END

-- vehicleController / Gearbox

local gearbox = powertrain.getDevice("gearbox")
local controlLogicName = "dummy"
if gearbox then
  controlLogicName = gearbox.type
end

--if jbeamData.shiftLogicName then
--  controlLogicName = jbeamData.shiftLogicName
--end
  
local controlLogicModuleName = "controller/shiftLogic-" .. controlLogicName

local controlModule = require(controlLogicModuleName)

local gears = {
	["P"] = 0,
	["R"] = 7,
	["N"] = 6,
	["D"] = 5,
	["1"] = 1,
	["2"] = 2,
	["3"] = 3,
	[-1] = 7,
	[0] = 6,
	[1] = 1,
	[2] = 2,
	[3] = 3
}

-- END

local M = {}

local ip = "127.0.0.1"
local port = 4444

local udpSocket = nil

local ffi = require("ffi")

local function declareOutgaugeStruct()
  -- the documentation can be found at LFS/docs/InSim.txt
  ffi.cdef [[
  typedef struct outgauge_t  {
      unsigned       time;            // time in milliseconds (to check order)
      char           car[4];          // Car name
      unsigned short flags;           // Info (see OG_x below)
      char           gear;            // Reverse:0, Neutral:1, First:2...
      char           plid;            // Unique ID of viewed player (0 = none)
      float          speed;           // M/S
      float          rpm;             // RPM
      float          turbo;           // BAR
      float          engTemp;         // C
      float          fuel;            // 0 to 1
      float          oilPressure;     // BAR
      float          oilTemp;         // C
      unsigned       dashLights;      // Dash lights available (see DL_x below)
      unsigned       showLights;      // Dash lights currently switched on
      float          throttle;        // 0 to 1
      float          brake;           // 0 to 1
      float          clutch;          // 0 to 1
      char           display1[16];    // Usually Fuel
      char           display2[16];    // Usually Settings
      int            id;              // optional - only if OutGauge ID is specified
  } outgauge_t;
  ]]
end
pcall(declareOutgaugeStruct)

--[[
CONSTANTS
// OG_x - bits for OutGaugePack Flags
#define OG_SHIFT      1        // key
#define OG_CTRL       2        // key
#define OG_TURBO      8192     // show turbo gauge
#define OG_KM         16384    // if not set - user prefers MILES
#define OG_BAR        32768    // if not set - user prefers PSI

// DL_x - bits for OutGaugePack DashLights and ShowLights
DL_SHIFT,           // bit 0    - shift light
DL_FULLBEAM,        // bit 1    - full beam
DL_HANDBRAKE,       // bit 2    - handbrake
DL_PITSPEED,        // bit 3    - pit speed limiter
DL_TC,              // bit 4    - TC active or switched off
DL_SIGNAL_L,        // bit 5    - left turn signal
DL_SIGNAL_R,        // bit 6    - right turn signal
DL_SIGNAL_ANY,      // bit 7    - shared turn signal
DL_OILWARN,         // bit 8    - oil pressure warning
DL_BATTERY,         // bit 9    - battery warning
DL_ABS,             // bit 10   - ABS active or switched off
DL_ESC,           	// bit 11	- ESC
DL_HEADLIGHT,		// bit 12	- Headlights on or off
DL_CHECK_ENGINE,	// bit 13	- Check Engine
DL_ESC_OFF,			// bit 14	- ESC turned off
DL_FOGLIGHTS		// bit 15	- Fog lights on or off
DL_CRUISE_MAIN		// bit 16	- Cruise main
DL_CRUISE_SET		// bit 17	- Cruise set
DL_DOOR_OPEN		// bit 18	- Door open
DL_TRUNK_OPEN		// bit 19	- Trunk open
]]
local OG_KM = 16384
local OG_BAR = 32768
local OG_TURBO = 8192

local DL_SHIFT = 2 ^ 0
local DL_FULLBEAM = 2 ^ 1
local DL_HANDBRAKE = 2 ^ 2
local DL_TC = 2 ^ 4
local DL_SIGNAL_L = 2 ^ 5
local DL_SIGNAL_R = 2 ^ 6
local DL_OILWARN = 2 ^ 8
local DL_BATTERY = 2 ^ 9
local DL_ABS = 2 ^ 10
local DL_ESC = 2 ^ 11
local DL_HEADLIGHT = 2 ^ 12
local DL_CHECK_ENGINE = 2 ^ 13
local DL_ESC_OFF = 2 ^ 14
local DL_FOGLIGHTS = 2 ^ 15
local DL_CRUISE_MAIN = 2 ^ 16
local DL_CRUISE_SET = 2 ^ 17
local DL_DOOR_OPEN = 2 ^ 18
local DL_TRUNK_OPEN = 2 ^ 19
local DL_TPMS = 2 ^ 20
local DL_TREAD = 2 ^ 21
local IGN = 2 ^ 24

local hasESC = false
local hasShiftLights = false

local function sendPackage(ip, port, id)
  --log('D', 'outgauge', 'sendPackage: '..tostring(ip) .. ':' .. tostring(port))

  if not electrics.values.watertemp then
    -- vehicle not completly initialized, skip sending package
    return
  end

  local o = ffi.new("outgauge_t")
  -- set the values
  o.time = 0 -- not used atm
  o.car = "beam"
  o.flags = OG_KM + OG_BAR + (electrics.values.turboBoost and OG_TURBO or 0)
  
  local gearToSet = 0
  if controlModule.getGearName() then
    if type(controlModule.getGearName()) == "number" then
	  gearToSet = gears[controlModule.getGearName()]
	elseif controlModule.getGearName():sub(0,0) == "S" then
	  gearToSet = gears["D"]
	else
	  gearToSet = gears[controlModule.getGearName():sub(-1, -1)]
	end
	
	if not gearToSet then
	  o.gear = 255
	else
	  o.gear = gearToSet
	end
  else
	o.gear = 255
  end
  
  o.plid = 0
  o.speed = electrics.values.wheelspeed or electrics.values.airspeed
  o.rpm = electrics.values.rpm or 0
  o.turbo = (electrics.values.turboBoost or 0) / 14.504

  o.engTemp = electrics.values.watertemp or 0
  o.fuel = electrics.values.fuel or 0
  o.oilPressure = 0 -- TODO
  o.oilTemp = electrics.values.oiltemp or 0

  -- the lights
  o.dashLights = bit.bor(o.dashLights, DL_FULLBEAM)
  if electrics.values.highbeam ~= 0 then
    o.showLights = bit.bor(o.showLights, DL_FULLBEAM)
  end

  o.dashLights = bit.bor(o.dashLights, DL_HANDBRAKE)
  if electrics.values.parkingbrake ~= 0 then
    o.showLights = bit.bor(o.showLights, DL_HANDBRAKE)
  end

  o.dashLights = bit.bor(o.dashLights, DL_SIGNAL_L)
  if electrics.values.signal_L ~= 0 then
    o.showLights = bit.bor(o.showLights, DL_SIGNAL_L)
  end

  o.dashLights = bit.bor(o.dashLights, DL_SIGNAL_R)
  if electrics.values.signal_R ~= 0 then
    o.showLights = bit.bor(o.showLights, DL_SIGNAL_R)
  end

  local hasABS = electrics.values.hasABS or false
  if hasABS then
    o.dashLights = bit.bor(o.dashLights, DL_ABS)
    if electrics.values.abs ~= 0 then
      o.showLights = bit.bor(o.showLights, DL_ABS)
    end
  end

  o.dashLights = bit.bor(o.dashLights, DL_OILWARN)
  if electrics.values.oil ~= 0 then
    o.showLights = bit.bor(o.showLights, DL_OILWARN)
  end

  o.dashLights = bit.bor(o.dashLights, DL_BATTERY)
  if electrics.values.engineRunning == 0 then
    o.showLights = bit.bor(o.showLights, DL_BATTERY)
  end

  hasESC = electrics.values.hasESC
  if hasESC then
    o.dashLights = bit.bor(o.dashLights, DL_TC)
	o.dashLights = bit.bor(o.dashLights, DL_ESC)
    if electrics.values.tcs ~= 0 then
      o.showLights = bit.bor(o.showLights, DL_TC)
	end
	if electrics.values.esc ~= 0 then
	  o.showLights = bit.bor(o.showLights, DL_ESC)
    end
  elseif controller.getController('yawControl') ~= nil then
	if controller.getController('yawControl').isActing then
	  o.showLights = bit.bor(o.showLights, DL_ESC)
	end
  end

  if hasShiftLights then
    o.dashLights = bit.bor(o.dashLights, DL_SHIFT)
    if electrics.values.shouldShift then
      o.showLights = bit.bor(o.showLights, DL_SHIFT)
    end
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_HEADLIGHT)
  if electrics.values.lowhighbeam == 1 then
    o.showLights = bit.bor(o.showLights, DL_HEADLIGHT)
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_CHECK_ENGINE)
  if electrics.values.checkengine then
    o.showLights = bit.bor(o.showLights, DL_CHECK_ENGINE)
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_ESC_OFF)
  if controller.getController('esc') ~= nil then
    if not controller.getController('esc').getCurrentConfigData().escEnabled then
	  o.showLights = bit.bor(o.showLights, DL_ESC_OFF)
    end
  elseif controller.getController('yawControl') ~= nil then
	if not isEscEnabled then
	  o.showLights = bit.bor(o.showLights, DL_ESC_OFF)
	end
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_CRUISE_MAIN)
  o.dashLights = bit.bor(o.dashLights, DL_CRUISE_SET)
  if cruiseControl ~= nil then
    if cruiseControl.getConfiguration().isEnabled then
      o.showLights = bit.bor(o.showLights, DL_CRUISE_MAIN)
	  o.showLights = bit.bor(o.showLights, DL_CRUISE_SET)
    end
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_FOGLIGHTS)
  if electrics.values.fog == 1 then
    o.showLights = bit.bor(o.showLights, DL_FOGLIGHTS)
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_DOOR_OPEN)
  if getDoorsOpen() then
    o.showLights = bit.bor(o.showLights, DL_DOOR_OPEN)
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_TRUNK_OPEN)
  if getTrunkOpen() then
    o.showLights = bit.bor(o.showLights, DL_TRUNK_OPEN)
  end
  
  o.dashLights = bit.bor(o.dashLights, DL_TPMS)
  o.dashLights = bit.bor(o.dashLights, DL_TREAD)
  for _, wheel in ipairs(wheels.wheels) do
	if wheel.isTireDeflated then
	  o.showLights = bit.bor(o.showLights, DL_TREAD)
	  break
	end
  end
  
  o.dashLights = bit.bor(o.dashLights, IGN)
  o.showLights = bit.bor(o.showLights, IGN)

  o.throttle = electrics.values.throttle
  o.brake = electrics.values.brake
  o.clutch = electrics.values.clutch
  o.display1 = "" -- TODO
  o.display2 = "" -- TODO
  o.id = id

  local packet = ffi.string(o, ffi.sizeof(o)) --convert the struct into a string
  udpSocket:sendto(packet, ip, port)
  --log("I", "", "SendPackage for ID '"..dumps(id).."': "..dumps(electrics.values.rpm))
end

local function updateGFX(dt)
  if not playerInfo.firstPlayerSeated then
    return
  end
  sendPackage(ip, port, 0)
end

local function onExtensionLoaded()
  if not ffi then
    log("E", "outgauge", "Unable to load outgauge module: Lua FFI required")
    return false
  end

  if not udpSocket then
    udpSocket = socket.udp()
  end

  ip = settings.getValue("outgaugeIP")
  port = tonumber(settings.getValue("outgaugePort"))

  log("I", "", "Outgauge initialized for: " .. tostring(ip) .. ":" .. tostring(port))

  local shiftLightControllers = controller.getControllersByType("shiftLights")
  hasShiftLights = shiftLightControllers and #shiftLightControllers > 0
  return true
end

local function onExtensionUnloaded()
  if udpSocket then
    udpSocket:close()
  end
  udpSocket = nil
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.updateGFX = updateGFX

M.sendPackage = sendPackage

M.getDoorsOpen = getDoorsOpen
M.getTrunkOpen = getTrunkOpen

M.comp = comp
M.controlModule = controlModule

return M
