-- EOS R check (Lua on the R)
-- Runs the parts of ML's Lua API that should work on the EOS R and writes every result to ML/LOGS/LUATEST.LOG.
-- Changes nothing permanently: ISO is set to 400 and back only if it is manual. Takes no pictures.
-- Shows PASS/FAIL per line on the console; SET closes it.

menu.close()
console.show()

local log = io.open("ML/LOGS/LUATEST.LOG", "a")
local pass, fail = 0, 0
local function out(s)
    print(s)
    if log then log:write(s .. "\n"); log:flush() end
end
local function pad(s, n) s = tostring(s); return s .. string.rep(" ", n - #s) end  -- ML's format has no %-16s
local function check(name, fn)
    local ok, res = pcall(fn)
    if ok and res ~= false then
        pass = pass + 1
        out("PASS " .. pad(name, 12) .. tostring(res))
    else
        fail = fail + 1
        out("FAIL " .. pad(name, 12) .. tostring(res))
    end
end

local d = dryos.date
out(string.format("---- rtest %04d-%02d-%02d %02d:%02d:%02d", d.year, d.month, d.day, d.hour, d.min, d.sec))
check("camera", function() return camera.model .. " fw " .. camera.firmware end)
check("mode", function() return "mode " .. tostring(camera.mode) .. ", movie " .. tostring(lv.vidmode) end)
check("exposure", function()
    return string.format("ISO %s, %s s, f/%s", tostring(camera.iso.value), tostring(camera.shutter.value),
                         tostring(camera.aperture.value))
end)
check("lens", function() return tostring(lens.name) .. " " .. tostring(lens.focal_length) .. " mm" end)
check("battery", function()
    local ok, r = pcall(function() return battery.level end)
    if not ok then return "n/a on the R (" .. tostring(r):gsub("^.*: ", "") .. ")" end
    return tostring(r) .. " %"
end)
check("liveview", function() return "enabled " .. tostring(lv.enabled) .. ", running " .. tostring(lv.running) end)
check("card", function() return tostring(dryos.shooting_card.free_space) .. " MB free" end)

-- ISO write and restore (manual ISO only)
check("iso write", function()
    local old = camera.iso.value
    if old == nil or old == 0 then return "skipped (ISO Auto)" end
    camera.iso.value = 400
    msleep(300)
    local got = camera.iso.value
    camera.iso.value = old
    msleep(300)
    if got ~= 400 then error("set 400, read " .. tostring(got)) end
    if camera.iso.value ~= old then error("restore to " .. tostring(old) .. " failed: " .. tostring(camera.iso.value)) end
    return "400 ok, back to " .. tostring(old)
end)

-- lv.wait: 30 frames should take ~1 s (R: timed, 34 ms per frame)
check("lv.wait", function()
    if not lv.running then return "skipped (LV off)" end
    local t0 = dryos.ms_clock
    lv.wait(30)
    local ms = dryos.ms_clock - t0
    if ms < 800 or ms > 1600 then error(ms .. " ms for 30 frames") end
    return ms .. " ms for 30 frames"
end)

-- not supported on the R: must fail fast, not hang
check("lens.focus", function()
    if not lv.running then return "skipped (LV off)" end
    local t0 = dryos.ms_clock
    local ok, r = pcall(lens.focus, 1, 1)
    local ms = dryos.ms_clock - t0
    if ms > 500 then error("took " .. ms .. " ms") end
    if not ok then return "error '" .. tostring(r) .. "' in " .. ms .. " ms" end
    return "returned " .. tostring(r) .. " in " .. ms .. " ms (expected false)"
end)

check("menu", function() return "Global Draw = " .. tostring(menu.get("Overlay", "Global Draw")) end)
check("file", function()
    local f = io.open("ML/LOGS/LUATEST.TMP", "w"); f:write("abc123"); f:close()
    f = io.open("ML/LOGS/LUATEST.TMP", "r"); local s = f:read("*a"); f:close()
    dryos.remove("ML/LOGS/LUATEST.TMP")
    if s ~= "abc123" then error("read back '" .. tostring(s) .. "'") end
    return "write/read/remove ok"
end)
check("display", function()
    display.rect(20, 380, 300, 40, COLOR.WHITE, COLOR.BLUE)
    display.print("Lua draws on the R", 30, 388, FONT.MED, COLOR.WHITE, COLOR.BLUE)
    msleep(1500)
    return "drawn (look for a blue box)"
end)
check("memory", function() return string.format("Lua uses %d KB", math.floor(collectgarbage("count"))) end)

out(string.format("---- %d PASS, %d FAIL", pass, fail))
-- v4.13: the camera froze after this point once (v4.12). Log each exit step so the card shows where it stops.
out("exit: press SET (closes by itself after 20 s)")
local k = key.wait(KEY.SET, 20000)
out("exit: key.wait returned " .. tostring(k) .. (k == 0 and " (timeout: SET not seen)" or " (SET seen)"))
console.hide()
out("exit: console hidden, script ending")
if log then log:close() end
