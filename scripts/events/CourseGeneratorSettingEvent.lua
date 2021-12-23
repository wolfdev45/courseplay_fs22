--- @class CourseGeneratorSettingEvent
CourseGeneratorSettingEvent = {}

local CourseGeneratorSettingEvent_mt = Class(CourseGeneratorSettingEvent, Event)

InitEventClass(CourseGeneratorSettingEvent, "CourseGeneratorSettingEvent")

function CourseGeneratorSettingEvent.emptyNew()
	return Event.new(CourseGeneratorSettingEvent_mt)
end

--- Creates a new Event
function CourseGeneratorSettingEvent.new(vehicle,settingIx)
	local self = CourseGeneratorSettingEvent.emptyNew()
	self.vehicle = vehicle
	self.settingIx = settingIx
	return self
end

--- Reads the serialized data on the receiving end of the event.
function CourseGeneratorSettingEvent:readStream(streamId, connection) -- wird aufgerufen wenn mich ein Event erreicht
	self.vehicle = NetworkUtil.readNodeObject(streamId)
	local settings = self.vehicle:getCourseGeneratorSettingsTable()
	self.settingIx = streamReadUInt8(streamId)
	local setting = settings[self.settingIx]
	setting:readStream(streamId,connection)
	
--	self.currentIx = streamReadUInt16(streamId)
	self:run(connection);
end

--- Writes the serialized data from the sender.
function CourseGeneratorSettingEvent:writeStream(streamId, connection)  -- Wird aufgrufen wenn ich ein event verschicke (merke: reihenfolge der Daten muss mit der bei readStream uebereinstimmen 
	NetworkUtil.writeNodeObject(streamId,self.vehicle)
	streamWriteUInt8(streamId,self.settingIx)
	local settings = self.vehicle:getCourseGeneratorSettingsTable()
	local setting = settings[self.settingIx]
	setting:writeStream(streamId,connection)
--	streamWriteUInt16(streamId,self.currentIx)
end

--- Runs the event on the receiving end of the event.
function CourseGeneratorSettingEvent:run(connection) -- wir fuehren das empfangene event aus
	--- If the receiver was the client make sure every clients gets also updated.
	if not connection:getIsServer() then
		g_server:broadcastEvent(CourseGeneratorSettingEvent.new(self.vehicle,self.settingIx), nil, connection, self.vehicle)
	end
end

function CourseGeneratorSettingEvent.sendEvent(vehicle,settingIx)
	if g_server ~= nil then
		g_server:broadcastEvent(CourseGeneratorSettingEvent.new(vehicle,settingIx), nil, nil, vehicle)
	else
		g_client:getServerConnection():sendEvent(CourseGeneratorSettingEvent.new(vehicle,settingIx))
	end
end
