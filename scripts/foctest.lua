-- EOS R focus-step check (batch D: follow / rack focus)
-- Drives the lens a few steps near and far with each of ML's 3 step sizes and logs every result to
-- ML/LOGS/FOCTEST.LOG (time, lens.focus() result, focus position/distance before and after).
-- Needs: LiveView, lens switch on AF, Servo AF / Movie Servo AF off. Takes no pictures.
-- The lens ends roughly where it started (same number of steps each way). SET closes the console.

menu.close()
console.show()

local function out(s)
    print(s)
    local f = io.open("ML/LOGS/FOCTEST.LOG", "a")      -- open/append/close per line: a freeze keeps the earlier lines
    if f then f:write(s .. "\n"); f:close() end
end

local function state()
    local ok1, pos = pcall(function() return lens.focus_pos end)
    local ok2, dist = pcall(function() return lens.focus_distance end)
    return string.format("pos %s dist %s", tostring(ok1 and pos or "?"), tostring(ok2 and dist or "?"))
end

local d = dryos.date
out(string.format("---- foctest %04d-%02d-%02d %02d:%02d:%02d", d.year, d.month, d.day, d.hour, d.min, d.sec))
out(string.format("lens %s %s mm; af_mode %s, af %s; lv %s; %s", tostring(lens.name), tostring(lens.focal_length),
                  tostring(lens.af_mode), tostring(lens.af), tostring(lv.running), state()))

local function step(n, size, wait)
    local t0 = dryos.ms_clock
    local before = state()
    local ok, res = pcall(function() return lens.focus(n, size, wait) end)
    local dt = dryos.ms_clock - t0
    out(string.format("%6d focus(%d, %d, %s): %s %s in %d ms; %s -> %s", t0 % 1000000, n, size, tostring(wait),
                      ok and "returned" or "ERROR", tostring(res), dt, before, state()))
    msleep(300)
    return ok
end

if not lens.af then
    out("lens is in MF (or af_mode reads MF): switch the lens/camera to AF and run again")
else
    for size = 1, 3 do
        if not step(3, size, true) then break end
        step(-3, size, true)
    end
    out("-- open loop (no wait), 5 small steps each way")
    step(5, 1, false)
    step(-5, 1, false)
end
out("done")
print("SET closes this console")
