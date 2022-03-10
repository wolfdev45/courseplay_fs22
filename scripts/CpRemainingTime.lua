---@class CpRemainingTime
CpRemainingTime = CpObject()
--- Starting error estimate, gets less influence once the progress since the start increases.
CpRemainingTime.initialError = 0.05 -- +5%

function CpRemainingTime:init(vehicle, course, startIx)
	self.vehicle = vehicle
	self.startWpIx = startIx
	self.course = course
	self.length = course:getLength()
	self.timePassed = 0
	self.distanceTraveled = 0
	self.lastError = 0
	self.lastAverageSpeed = self:getOptimalSpeed()
	self.progress = 0
	self.firstEstimate = nil
end

function CpRemainingTime:delete()
	if self.course then
		local courseLengthTraveled = (self.course:getProgress(self.course:getLastPassedWaypointIx()) - self.course:getProgress(self.startWpIx)) * self.length / 1000
		self:debug("Time needed: %s, first time estimate: %s, course length since start wp: %.2fkm, distance traveled: %.2fkm, average speed: %.2fkm/h",
					CpGuiUtil.getFormatTimeText(self.timePassed), 
					CpGuiUtil.getFormatTimeText(self.firstEstimate),
					courseLengthTraveled,
					self.distanceTraveled / 1000,
					self.lastAverageSpeed * 3.6
					)
	end
end

function CpRemainingTime:debug(...)
    CpUtil.debugVehicle(CpDebug.DBG_FIELDWORK, self.vehicle, ...)
end

function CpRemainingTime:debugSparse(...)
    if g_updateLoopIndex % 100 == 0 then
        self:debug(...)
    end
end

function CpRemainingTime:update(dt)
	local wpIx = self.course:getCurrentWaypointIx()
	if wpIx < self.startWpIx then 
		wpIx = self.startWpIx
	end
	
	--- Makes sure the start error gets applied more at the beginning and less near the end.
	local progressSinceStart = MathUtil.clamp(self.course:getProgress(wpIx) - self.course:getProgress(self.startWpIx), 0, 1)
	self.progress = progressSinceStart

	--- Makes sure the calculated error gets applied more towards the end.
	local invertedProgressSinceStart = 1 - progressSinceStart

	--- Updates global values
	if self.vehicle:getLastSpeed() > 1 then 
		--- Distance traveled since start in m.
		self.distanceTraveled = self.distanceTraveled + self.vehicle.lastMovedDistance
		--- Actual time passed since the start, excludes time where the driver is standing still.
		self.timePassed = self.timePassed + dt/1000
	end

	if self.vehicle:getLastSpeed() > 3 then
				
		--- Actual distance traveled since start.
		local distanceTraveledSinceStart = self.distanceTraveled 
	
		--- Average speed in m/s
		self.lastAverageSpeed = MathUtil.clamp(distanceTraveledSinceStart/self.timePassed, 3/3.6, 50/3.6)
		self.lastEstimate = self:getPredicatedTimeRemaining(wpIx) * progressSinceStart + self:getOptimalTimeRemaining(wpIx) * invertedProgressSinceStart 

	else
		self.lastEstimate = self:getPredicatedTimeRemaining(wpIx) * progressSinceStart + self:getOptimalTimeRemaining(wpIx) * invertedProgressSinceStart 
	end
	if not self.firstEstimate then 
		self.firstEstimate = self.lastEstimate
	end
end

---------------------------------------------------------
--- Calculate the remaining course time left.
---------------------------------------------------------

function CpRemainingTime:getTimeRemaining()
	return self.lastEstimate
end

function CpRemainingTime:getRemainingCourseTime(ix, speed)
    local dx = self.course:getProgress(ix)
    return ((1-dx) * self.length)/speed -- in seconds
end

--- Gets the optimal time with a initial error applied.
function CpRemainingTime:getOptimalTimeRemaining(wpIx)
    return self:getRemainingCourseTime(wpIx, self:getOptimalSpeed()) * (1 + self.initialError)
end

--- Gets the predicated time relative to the speed limit.
function CpRemainingTime:getPredicatedTimeRemaining(wpIx)
    
    --- Time remaining of the course.
    local timeRemaining = self:getRemainingCourseTime(wpIx, self.lastAverageSpeed)
    --- Course time since starting point.
    local courseTimeSinceStart = self:getRemainingCourseTime(self.startWpIx, self.lastAverageSpeed)

    local courseTimeTraveled = courseTimeSinceStart - timeRemaining

    --- Absolute time error to the optimal course time traveled.
    local timeError = self.timePassed  - courseTimeTraveled
    
    --- Relative time error to the optimal course time traveled.
    self.lastError = MathUtil.clamp(timeError/courseTimeTraveled, -0.5, 0.5)
    return timeRemaining * (1 + self.lastError)
end

function CpRemainingTime:getOptimalSpeed()
	return MathUtil.clamp(self.vehicle:getSpeedLimit(true), 1, self.vehicle:getCpSettings().fieldWorkSpeed:getValue())/3.6 -- in m/s
end