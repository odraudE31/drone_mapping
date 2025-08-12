-- trigger_runcam_geolog.lua
local INTERVAL_MS = 10000
local last_time = 0

local function get_vehicle_loc()
  local pos = ahrs:get_position()
  if pos then
    local lat = pos:lat() / 1e7
    local lng = pos:lng() / 1e7
    local alt = pos:alt() / 100 -- altitude in meters AMSL
    return lat, lng, alt
  end
  return nil
end

local function take_photo()
  gcs:send_text(0, "Lua: Triggering photo")
  local ok, err = camera:take_picture()
  if not ok then
    gcs:send_text(1, "Error triggering camera: " .. tostring(err))
  end
  local lat, lng, alt = get_vehicle_loc()
  if lat then
    gcs:send_text(0, string.format("Loc: %.7f, %.7f; Alt: %.2f m AMSL", lat, lng, alt))
  else
    gcs:send_text(1, "GPS lock not available")
  end
end

local function loop()
  local t = time.time_ms()
  if t - last_time >= INTERVAL_MS then
    last_time = t
    take_photo()
  end
  return loop, 500
end

return loop()
