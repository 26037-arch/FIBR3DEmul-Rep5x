-- Rep5x scene builder, kinematic controller, and filament deposition.
-- No serial, USB, firmware, or real-printer API is used by this add-on.

local sim=require('sim')
local simUI=nil
if not sim.getBoolParam(sim.boolparam_headless) then simUI=require('simUI') end
-- CoppeliaSim 4.10 loads installed add-ons from an in-memory string, so
-- debug.getinfo() no longer exposes the source file's directory.
local scriptDir=sim.getStringParam(sim.stringparam_application_path)..'/addOns'
local profile=dofile(scriptDir..'/rep5x_profile.lua')

-- ---------------------------------------------------------------------------
-- Tunables. Everything below is appearance or deposition only; the kinematic
-- joint frames (Y bed, Z, X, C at [0,LC,LB], B at [0,-LC,0], tip at [0,0,-LB])
-- are unchanged from the phase-1 contract.
-- ---------------------------------------------------------------------------
local EXT={
    filamentDiameterMm=1.75,
    beadWidthMm=0.45,      -- rendering width when E is absent/unusable
    minBeadMm=0.15,
    maxBeadMm=1.20,
    dotSpacingMm=0.25,     -- bead sample spacing along the path
    maxPoints=400000,      -- drawing-object capacity (≈100 m of path)
    color={0.10,0.55,0.95},
    defaultFeedMmMin=1500, -- G1 without F
    rapidFeedMmMin=6000,   -- G0 (travel) feed
    rotaryDegPerS=90,      -- pure B/C moves
    firstLayerRatio=1.0,
}

local COL={
    extrusion={0.13,0.13,0.14}, steel={0.17,0.17,0.18}, alu={0.74,0.76,0.78},
    printed={0.93,0.42,0.08}, motor={0.10,0.10,0.11}, motorFace={0.62,0.64,0.66},
    brass={0.80,0.62,0.22}, belt={0.07,0.07,0.08}, wheel={0.05,0.05,0.06},
    sheet={0.09,0.09,0.10}, screen={0.06,0.10,0.18}, rubber={0.08,0.08,0.08},
    sock={0.20,0.45,0.90}, ptfe={0.92,0.93,0.95}, wire={0.12,0.12,0.13},
    fan={0.11,0.11,0.12}, spool={0.20,0.20,0.22}, filament=nil,
}
COL.filament=EXT.color
local SPEC={0.55,0.55,0.55}

local state={
    ui=nil,handles={},moves={},index=1,running=false,paused=false,speed=1,
    pose={X=0,Y=0,Z=0,C=0,B=0},mode='idle',selfTestPath=nil,selfTestWritten=false,
    seg=nil,
    extrusion={handle=nil,points=0,filamentMm=0,perItemSize=false,full=false,first=nil,last=nil},
    bowden={handle=nil,disabled=false,outlet=nil,inlet=nil},
    meltDrop=nil,meltVisible=false,mismatchWarned=false,
}

local function jsonString(value)
    return '"'..tostring(value):gsub('\\','\\\\'):gsub('"','\\"'):gsub('\r','\\r'):gsub('\n','\\n')..'"'
end

local function vec3Json(v)
    if not v then return 'null' end
    return string.format('[%.12g, %.12g, %.12g]',v[1],v[2],v[3])
end

local function writeSelfTest(ok,message)
    if not state.selfTestPath or state.selfTestWritten then return end
    state.selfTestWritten=true
    local tip={0,0,0}
    if state.handles[profile.objects.tip] then tip=sim.getObjectPosition(state.handles[profile.objects.tip],-1) end
    local tipBed={0,0,0}
    if state.handles[profile.objects.tip] and state.handles[profile.objects.drawBoard] then
        local bedSurfaceWorld=sim.getObjectPosition(state.handles[profile.objects.drawBoard],-1)
        tipBed={tip[1]-bedSurfaceWorld[1],tip[2]-bedSurfaceWorld[2],tip[3]-bedSurfaceWorld[3]}
    end
    local joints={}
    for _,axis in ipairs({'X','Y','Z','C','B'}) do
        local h=state.handles[profile.joints[axis]]
        joints[axis]=h and sim.getJointPosition(h) or 0
    end
    local p=state.pose
    local e=state.extrusion
    local payload=string.format(
        '{\n  "ok": %s,\n  "message": %s,\n  "coppelia_version": %s,\n  "moves_completed": %d,\n  "final_pose_mm_deg": {"X": %.9g, "Y": %.9g, "Z": %.9g, "C": %.9g, "B": %.9g},\n  "scene_joints_m_rad": {"X": %.12g, "Y": %.12g, "Z": %.12g, "C": %.12g, "B": %.12g},\n  "nozzle_world_mm": [%.12g, %.12g, %.12g],\n  "tcp_relative_bed_mm": [%.12g, %.12g, %.12g],\n  "extrusion": {"points": %d, "filament_mm": %.9g, "first_point_bed_mm": %s, "last_point_bed_mm": %s}\n}\n',
        tostring(ok),jsonString(message),jsonString(sim.getInt32Param(sim.intparam_program_full_version)),math.max(0,state.index-1),
        p.X,p.Y,p.Z,p.C,p.B,joints.X,joints.Y,joints.Z,joints.C,joints.B,tip[1]*1000,tip[2]*1000,tip[3]*1000,tipBed[1]*1000,tipBed[2]*1000,tipBed[3]*1000,
        e.points,e.filamentMm,vec3Json(e.first),vec3Json(e.last))
    local f,err=io.open(state.selfTestPath,'w')
    if not f then sim.addLog(sim.verbosity_scripterrors,'[Rep5x] cannot write self-test result: '..tostring(err)); return end
    f:write(payload)
    f:close()
    sim.addLog(sim.verbosity_scriptinfos,'[Rep5x] self-test result written to '..state.selfTestPath)
end

local function status(text)
    state.mode=text
    if state.ui then simUI.setLabelText(state.ui,201,text) end
    sim.addLog(sim.verbosity_scriptinfos,'[Rep5x] '..text)
end

local function fail(text)
    state.running=false
    state.paused=false
    state.seg=nil
    status('ERROR: '..text)
    sim.addLog(sim.verbosity_scripterrors,'[Rep5x] '..text)
    writeSelfTest(false,text)
end

-- ---------------------------------------------------------------------------
-- Scene construction helpers
-- ---------------------------------------------------------------------------
local function setAlias(handle,name)
    sim.setObjectAlias(handle,name)
end

local function child(handle,parent,pos,ori)
    sim.setObjectParent(handle,parent,false)
    sim.setObjectPosition(handle,parent,pos or {0,0,0})
    sim.setObjectOrientation(handle,parent,ori or {0,0,0})
    return handle
end

local function paint(h,color,spec)
    sim.setShapeColor(h,nil,sim.colorcomponent_ambient_diffuse,color or {0.45,0.48,0.52})
    if spec then pcall(sim.setShapeColor,h,nil,sim.colorcomponent_specular,spec) end
end

local function prim(name,kind,size,parent,pos,ori,color,spec)
    local h=sim.createPrimitiveShape(kind,size,0)
    setAlias(h,name)
    child(h,parent,pos,ori)
    sim.setObjectInt32Param(h,sim.shapeintparam_static,1)
    pcall(sim.setObjectInt32Param,h,sim.shapeintparam_respondable,0)
    paint(h,color,spec)
    return h
end

-- Axis-aligned cuboid (size = {x,y,z}).
local function box(name,size,parent,pos,color,ori,spec)
    return prim(name,sim.primitiveshape_cuboid,size,parent,pos,ori,color,spec)
end

-- Cylinder of diameter d and height h. axis: 'z' (default), 'x', or 'y'.
local AXIS_ORI={z={0,0,0},x={0,math.pi/2,0},y={-math.pi/2,0,0}}
local function cyl(name,d,h,parent,pos,color,axis,spec)
    return prim(name,sim.primitiveshape_cylinder,{d,d,h},parent,pos,AXIS_ORI[axis or 'z'],color,spec)
end

local function sphere(name,d,parent,pos,color,spec)
    return prim(name,sim.primitiveshape_spheroid,{d,d,d},parent,pos,{0,0,0},color,spec)
end

local function cross(a,b) return {a[2]*b[3]-a[3]*b[2],a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]} end
local function norm(a)
    local l=math.sqrt(a[1]*a[1]+a[2]*a[2]+a[3]*a[3])
    return {a[1]/l,a[2]/l,a[3]/l},l
end

-- Euler angles (CoppeliaSim alpha/beta/gamma, R = Rx(a)·Ry(b)·Rz(g)) of a frame
-- whose columns are the unit axes x,y,z.
local function eulerFromAxes(x,y,z)
    local b=math.asin(math.max(-1,math.min(1,z[1])))
    local a=math.atan(-z[2],z[3])
    local g=math.atan(-y[1],x[1])
    return {a,b,g}
end

-- Straight member between points a and b (parent-frame coordinates).
-- kind 'cyl' (default) uses diameter d; kind 'slab' uses cross-section {d,w}.
local function member(name,parent,a,b,d,color,opts)
    opts=opts or {}
    local dir,len=norm({b[1]-a[1],b[2]-a[2],b[3]-a[3]})
    local hint=opts.hint or ((math.abs(dir[3])>0.9) and {0,1,0} or {0,0,1})
    local x=norm(cross(hint,dir))
    local y=cross(dir,x)
    local ori=eulerFromAxes(x,y,dir)
    local mid={(a[1]+b[1])/2,(a[2]+b[2])/2,(a[3]+b[3])/2}
    local kind=(opts.kind=='slab') and sim.primitiveshape_cuboid or sim.primitiveshape_cylinder
    local size={d,opts.w or d,len}
    return prim(name,kind,size,parent,mid,ori,color,opts.spec)
end

-- NEMA stepper: square body with a round boss and shaft on the -axis face.
-- 'face' is the unit vector (in parent frame) the shaft points to.
local function stepper(name,parent,center,sizeXY,length,face,shaftLen)
    local bodySize={sizeXY,sizeXY,length}
    local axis='z'
    if face[1]~=0 then bodySize={length,sizeXY,sizeXY}; axis='x'
    elseif face[2]~=0 then bodySize={sizeXY,length,sizeXY}; axis='y' end
    local body=box(name,bodySize,parent,center,COL.motor,nil,{0.25,0.25,0.25})
    local function along(k) return {center[1]+face[1]*k,center[2]+face[2]*k,center[3]+face[3]*k} end
    cyl(name..'_boss',sizeXY*0.52,0.002,parent,along(length/2+0.001),COL.motorFace,axis,SPEC)
    local sl=shaftLen or 0.012
    if sl>0 then cyl(name..'_shaft',0.005,sl,parent,along(length/2+sl/2),COL.alu,axis,SPEC) end
    return body
end

-- V-slot wheel (POM) with axis along 'axis'.
local function vwheel(name,parent,pos,axis)
    local w=cyl(name,0.024,0.011,parent,pos,COL.wheel,axis,{0.3,0.3,0.3})
    return w
end

local function joint(name,kind,parent,pos,ori,cyclic,interval)
    local h=sim.createJoint(kind,sim.jointmode_passive,0,{0.04,0.012})
    setAlias(h,name)
    child(h,parent,pos,ori)
    sim.setJointInterval(h,cyclic,interval)
    return h
end

local function findExact(path)
    local ok,h=pcall(sim.getObject,path)
    if ok then return h end
    return nil
end

local function validateHandles()
    local required={profile.joints.X,profile.joints.Y,profile.joints.Z,profile.joints.C,profile.joints.B,profile.objects.tip,profile.objects.bed,profile.objects.drawBoard}
    local seen={}
    local root=state.handles.root or findExact('/'..profile.root)
    if not root then error('missing required scene root '..profile.root) end
    local objects=sim.getObjectsInTree(root,sim.handle_all,0)
    for _,name in ipairs(required) do
        local matches={}
        for _,candidate in ipairs(objects) do
            if sim.getObjectAlias(candidate)==name then table.insert(matches,candidate) end
        end
        if #matches==0 then error('missing required scene object '..name) end
        if #matches>1 then error('duplicate scene alias '..name..' under '..profile.root) end
        local h=matches[1]
        if seen[h] then error('duplicate required scene handle for '..name..' and '..seen[h]) end
        seen[h]=name
        state.handles[name]=h
    end
    -- Optional cosmetic handles.
    for _,candidate in ipairs(objects) do
        local alias=sim.getObjectAlias(candidate)
        if alias=='Rep5x_MeltDrop' then state.meltDrop=candidate end
        if alias=='Rep5x_BowdenOutlet' then state.bowden.outlet=candidate end
        if alias=='Rep5x_BowdenInlet' then state.bowden.inlet=candidate end
    end
end

-- ---------------------------------------------------------------------------
-- Geometry (metres, world frame at the zero pose; the nozzle tip is the origin,
-- the G-code origin is the front-left corner of the 220 x 220 print area).
-- ---------------------------------------------------------------------------
local G={
    printArea=0.220, plate=0.235, plateT=0.001, bedT=0.003,
    bedCX=0.110, bedCY=0.110,         -- print-area centre relative to the G-code origin
    uprightX={-0.080,0.300}, uprightY=0.085, uprightZ0=-0.060, uprightZ1=0.380,
    baseZ=-0.080, baseY0=-0.170, baseY1=0.190,
    railY=0.010, railLen=0.400, railZ=-0.040,
    gantryZ=0.115, gantryY=0.045, gantryX0=-0.055, gantryX1=0.280,
    leadX=-0.080, leadY=0.045,
    topZ=0.400,
}

local function buildFrame(root)
    local frame=sim.createDummy(0.01)
    setAlias(frame,profile.objects.frame)
    child(frame,root)
    local cx=(G.uprightX[1]+G.uprightX[2])/2
    local width=G.uprightX[2]-G.uprightX[1]+0.040
    local depth=G.baseY1-G.baseY0+0.040
    -- Base: two 4040 side rails along Y, front/back 4040 cross members, feet.
    for i,x in ipairs(G.uprightX) do
        box('Rep5x_BaseRail'..i,{0.040,depth,0.040},frame,{x,(G.baseY0+G.baseY1)/2,G.baseZ},COL.extrusion)
        for j,y in ipairs({G.baseY0,G.baseY1}) do
            cyl('Rep5x_Foot'..i..j,0.022,0.012,frame,{x,y,G.baseZ-0.026},COL.rubber)
        end
    end
    box('Rep5x_BaseFront',{width-0.080,0.040,0.040},frame,{cx,G.baseY0,G.baseZ},COL.extrusion)
    box('Rep5x_BaseBack',{width-0.080,0.040,0.040},frame,{cx,G.baseY1,G.baseZ},COL.extrusion)
    -- Y linear rail (2040) on the cross members, Y stepper at the rear, belt idler at the front.
    box('Rep5x_YRail',{0.020,G.railLen,0.040},frame,{G.bedCX,G.railY,G.railZ},COL.extrusion)
    stepper('Rep5x_YStepper',frame,{G.bedCX,G.railY+G.railLen/2+0.024,G.railZ},0.042,0.040,{0,-1,0},0.010)
    box('Rep5x_YIdler',{0.020,0.018,0.030},frame,{G.bedCX,G.railY-G.railLen/2-0.006,G.railZ+0.004},COL.steel)
    -- Uprights (4040), top bar (4020), corner gussets.
    local uh=G.uprightZ1-G.uprightZ0
    for i,x in ipairs(G.uprightX) do
        box('Rep5x_Upright'..i,{0.040,0.040,uh},frame,{x,G.uprightY,(G.uprightZ0+G.uprightZ1)/2},COL.extrusion)
        box('Rep5x_Gusset'..i,{0.040,0.030,0.030},frame,{x,G.uprightY-0.035,G.uprightZ0+0.015},COL.steel)
    end
    box('Rep5x_TopBar',{width,0.020,0.040},frame,{cx,G.uprightY,G.topZ},COL.extrusion)
    -- Z drive: NEMA17 at the base of the left upright, 8 mm lead screw, coupler.
    stepper('Rep5x_ZStepper',frame,{G.leadX,G.leadY,-0.030},0.042,0.048,{0,0,1},0.0)
    cyl('Rep5x_ZCoupler',0.019,0.024,frame,{G.leadX,G.leadY,0.006},COL.alu,'z',SPEC)
    cyl('Rep5x_ZLeadScrew',0.008,0.350,frame,{G.leadX,G.leadY,0.018+0.175},COL.alu,'z',SPEC)
    -- Display (front right) and PSU (right side).
    box('Rep5x_DisplayMount',{0.030,0.020,0.030},frame,{0.265,G.baseY0-0.025,G.baseZ-0.004},COL.steel)
    local disp=box('Rep5x_Display',{0.105,0.012,0.070},frame,{0.265,G.baseY0-0.040,G.baseZ+0.020},COL.steel,{math.rad(-25),0,0})
    box('Rep5x_DisplayScreen',{0.085,0.002,0.050},disp,{0,-0.007,0.004},COL.screen,nil,{0.4,0.4,0.4})
    box('Rep5x_PSU',{0.060,0.220,0.045},frame,{G.uprightX[2]+0.045,G.railY,G.baseZ},COL.steel)
    box('Rep5x_PSUGrille',{0.002,0.180,0.030},frame,{G.uprightX[2]+0.0755,G.railY,G.baseZ},COL.motor)
    -- Rep5x external Bowden extruder on the top-left of the frame.
    local ex={-0.050,G.uprightY,0.450}
    box('Rep5x_ExtruderMount',{0.050,0.006,0.060},frame,{ex[1],ex[2],ex[3]},COL.printed)
    stepper('Rep5x_ExtruderStepper',frame,{ex[1],ex[2]+0.024,ex[3]+0.005},0.042,0.040,{0,-1,0},0.0)
    box('Rep5x_ExtruderBody',{0.040,0.030,0.044},frame,{ex[1],ex[2]-0.018,ex[3]+0.005},COL.alu,nil,SPEC)
    cyl('Rep5x_ExtruderGear',0.011,0.004,frame,{ex[1]+0.006,ex[2]-0.035,ex[3]+0.005},COL.brass,'y',SPEC)
    cyl('Rep5x_ExtruderFitting',0.008,0.008,frame,{ex[1],ex[2]-0.018,ex[3]-0.021},COL.alu,'z',SPEC)
    cyl('Rep5x_ExtruderInlet',0.006,0.006,frame,{ex[1],ex[2]-0.018,ex[3]+0.030},COL.alu,'z',SPEC)
    local outlet=sim.createDummy(0.004)
    setAlias(outlet,'Rep5x_BowdenOutlet')
    child(outlet,frame,{ex[1],ex[2]-0.018,ex[3]-0.025})
    -- Spool holder and filament spool above the top bar.
    local sp={0.150,G.uprightY,0.535}
    box('Rep5x_SpoolArm',{0.020,0.020,sp[3]-G.topZ-0.020},frame,{sp[1],sp[2],(sp[3]+G.topZ+0.020)/2},COL.steel)
    cyl('Rep5x_SpoolAxle',0.016,0.100,frame,sp,COL.steel,'y',SPEC)
    cyl('Rep5x_SpoolCore',0.056,0.062,frame,sp,COL.spool,'y')
    cyl('Rep5x_SpoolFlangeA',0.200,0.004,frame,{sp[1],sp[2]-0.031,sp[3]},COL.spool,'y')
    cyl('Rep5x_SpoolFlangeB',0.200,0.004,frame,{sp[1],sp[2]+0.031,sp[3]},COL.spool,'y')
    cyl('Rep5x_SpoolFilament',0.172,0.058,frame,sp,COL.filament,'y')
    member('Rep5x_FilamentStrand',frame,{sp[1]-0.020,sp[2],sp[3]-0.085},{ex[1],ex[2]-0.018,ex[3]+0.034},0.00175,COL.filament)
    return frame
end

local function buildBed(root)
    local yj=joint(profile.joints.Y,sim.joint_prismatic,root,{0,0,0},{-math.pi/2,0,0},false,{-profile.limits.Y[2],profile.limits.Y[2]-profile.limits.Y[1]})
    -- Y joint frame: local X = world X, local Z = world +Y, local Y = world -Z.
    local function toY(w) return {w[1],-w[3],w[2]} end
    local bedC={G.bedCX,G.bedCY,-G.plateT-G.bedT/2}
    local bed=box(profile.objects.bed,{G.plate,G.plate,G.bedT},yj,toY(bedC),COL.alu,{math.pi/2,0,0},SPEC)
    -- Children of the bed are world-aligned; convert world coordinates to bed-relative.
    local function rel(w) return {w[1]-bedC[1],w[2]-bedC[2],w[3]-bedC[3]} end
    box('Rep5x_PrintSheet',{G.plate,G.plate,G.plateT},bed,rel({G.bedCX,G.bedCY,-G.plateT/2}),COL.sheet,nil,{0.2,0.2,0.2})
    box('Rep5x_SheetTab',{0.030,0.012,G.plateT},bed,rel({G.bedCX,G.bedCY-G.plate/2-0.006,-G.plateT/2}),COL.sheet)
    -- G-code origin: the DrawBoard dummy marks TCP (0,0,0) on the print surface.
    local board=sim.createDummy(0.004)
    setAlias(board,profile.objects.drawBoard)
    child(board,bed,rel({0,0,0}))
    -- Bed carriage, springs, levelling knobs, V-wheels on the Y rail.
    box('Rep5x_BedCarriage',{G.plate,0.200,0.003},bed,rel({G.bedCX,G.bedCY,-0.0135}),COL.steel)
    for i,dx in ipairs({-0.100,0.100}) do
        for j,dy in ipairs({-0.100,0.100}) do
            cyl('Rep5x_BedSpring'..i..j,0.008,0.008,bed,rel({G.bedCX+dx,G.bedCY+dy,-0.008}),COL.brass,'z')
            cyl('Rep5x_BedKnob'..i..j,0.016,0.005,bed,rel({G.bedCX+dx,G.bedCY+dy,-0.0175}),COL.motor,'z')
        end
    end
    for i,dx in ipairs({-0.0155,0.0155}) do
        for j,dy in ipairs({-0.060,0.060}) do
            vwheel('Rep5x_BedWheel'..i..j,bed,rel({G.bedCX+dx,G.bedCY+dy,-0.027}),'x')
        end
    end
    box('Rep5x_BedBeltClamp',{0.012,0.030,0.010},bed,rel({G.bedCX,G.bedCY,-0.020}),COL.steel)
    return yj,bed,board
end

local function buildGantry(root)
    local z=joint(profile.joints.Z,sim.joint_prismatic,root,{0,0,0},{0,0,0},false,{profile.limits.Z[1],profile.limits.Z[2]-profile.limits.Z[1]})
    local gc={(G.gantryX0+G.gantryX1)/2,G.gantryY,G.gantryZ}
    local gantry=box(profile.objects.gantry,{G.gantryX1-G.gantryX0,0.020,0.020},z,gc,COL.extrusion)
    local function rel(w) return {w[1]-gc[1],w[2]-gc[2],w[3]-gc[3]} end
    -- Left Z bracket: plate, lead nut, V-wheels on the left upright, X stepper.
    box('Rep5x_ZBracketL',{0.070,0.004,0.090},gantry,rel({G.leadX,G.uprightY-0.024,G.gantryZ}),COL.steel)
    cyl('Rep5x_ZLeadNut',0.022,0.010,gantry,rel({G.leadX,G.leadY,G.gantryZ}),COL.brass,'z',SPEC)
    vwheel('Rep5x_ZWheelL1',gantry,rel({G.uprightX[1]-0.0255,G.uprightY,G.gantryZ+0.030}),'y')
    vwheel('Rep5x_ZWheelL2',gantry,rel({G.uprightX[1]+0.0255,G.uprightY,G.gantryZ+0.030}),'y')
    vwheel('Rep5x_ZWheelL3',gantry,rel({G.uprightX[1]-0.0255,G.uprightY,G.gantryZ-0.030}),'y')
    stepper('Rep5x_XStepper',gantry,rel({G.gantryX0+0.012,G.gantryY,G.gantryZ+0.031}),0.042,0.042,{0,0,-1},0.0)
    -- Right Z bracket with wheels.
    box('Rep5x_ZBracketR',{0.070,0.004,0.090},gantry,rel({G.uprightX[2],G.uprightY-0.024,G.gantryZ}),COL.steel)
    vwheel('Rep5x_ZWheelR1',gantry,rel({G.uprightX[2]-0.0255,G.uprightY,G.gantryZ+0.030}),'y')
    vwheel('Rep5x_ZWheelR2',gantry,rel({G.uprightX[2]+0.0255,G.uprightY,G.gantryZ+0.030}),'y')
    vwheel('Rep5x_ZWheelR3',gantry,rel({G.uprightX[2]+0.0255,G.uprightY,G.gantryZ-0.030}),'y')
    -- X belt in the front slot and the right-end idler.
    box('Rep5x_XBelt',{G.gantryX1-G.gantryX0-0.010,0.0015,0.006},gantry,rel({gc[1],G.gantryY-0.0115,G.gantryZ}),COL.belt)
    cyl('Rep5x_XIdler',0.012,0.014,gantry,rel({G.gantryX1-0.008,G.gantryY-0.006,G.gantryZ}),COL.alu,'z',SPEC)
    -- X joint: local Z = world X; the carriage is re-aligned to the world frame.
    local x=joint(profile.joints.X,sim.joint_prismatic,gantry,rel({0,0,0}),{0,math.pi/2,0},false,{profile.limits.X[1],profile.limits.X[2]-profile.limits.X[1]})
    return z,gantry,x
end

local function buildCarriage(x)
    -- X joint frame: local Z = world X, local X = world -Z. The carriage alias is the
    -- steel carriage plate, re-aligned to the world frame; world offset (wx,wy,wz)
    -- from the zero-pose tip becomes joint-local (-wz,wy,wx).
    local pc={0.035,0.0315,0.108}
    local carriage=box(profile.objects.carriage,{0.110,0.003,0.070},x,{-pc[3],pc[2],pc[1]},COL.steel,{0,-math.pi/2,0})
    local function rel(w) return {w[1]-pc[1],w[2]-pc[2],w[3]-pc[3]} end
    vwheel('Rep5x_XWheel1',carriage,rel({-0.010,G.gantryY,G.gantryZ+0.0185}),'y')
    vwheel('Rep5x_XWheel2',carriage,rel({0.080,G.gantryY,G.gantryZ+0.0185}),'y')
    vwheel('Rep5x_XWheel3',carriage,rel({0.035,G.gantryY,G.gantryZ-0.0185}),'y')
    -- Rep5x carriage mount (printed) with the C-axis bearing housing.
    box('Rep5x_CarriageMount',{0.120,0.005,0.058},carriage,rel({0.035,0.0265,0.104}),COL.printed)
    box('Rep5x_CBearingHousing',{0.044,0.034,0.010},carriage,rel({0,profile.lc+0.010,0.1305}),COL.printed)
    cyl('Rep5x_CBearing',0.022,0.006,carriage,rel({0,profile.lc,0.1225}),COL.alu,'z',SPEC)
    -- C stepper hangs in front of the mount; its pulley shares the C belt plane.
    local beltZ=profile.lb+0.040
    local cm={0.072,0,0.125}
    box('Rep5x_CMotorBracket',{0.046,0.032,0.005},carriage,rel({cm[1],0.0085,cm[3]+0.0225}),COL.printed)
    stepper('Rep5x_CStepper',carriage,rel(cm),0.042,0.040,{0,0,-1},0.0)
    cyl('Rep5x_CMotorPulley',0.012,0.010,carriage,rel({cm[1],cm[2],beltZ}),COL.alu,'z',SPEC)
    for i,s in ipairs({-1,1}) do
        member('Rep5x_CBelt'..i,carriage,rel({cm[1],cm[2]+s*0.0055,beltZ}),rel({0,profile.lc+s*0.0200,beltZ}),0.0015,COL.belt,{kind='slab',w=0.006,hint={0,0,1}})
    end
    -- Slip-ring wiring loom from the bearing housing up to the carriage.
    member('Rep5x_HeadLoom',carriage,rel({0,profile.lc,0.1355}),rel({0.030,0.030,0.150}),0.006,COL.wire)
    return carriage,rel
end

local function buildHead(carriage,crel)
    -- C pivot is +LC in Y and +LB in Z from the zero-pose TCP.
    local c=joint(profile.joints.C,sim.joint_revolute,carriage,crel({0,profile.lc,profile.lb}),{0,0,0},true,{0,2*math.pi})
    -- The C link alias is the C driven pulley. Its children are placed relative
    -- to the B pivot P=[0,-LC,0] (C frame), which is world aligned at C=0.
    local lc=profile.lc
    -- Pulley centre sits on the C axis, 40 mm above P: C-frame [0,0,0.040] = P + [0,+LC,0.040].
    local cLink=cyl(profile.objects.cLink,0.041,0.012,c,{0,0,0.040},COL.steel,'z',{0.3,0.3,0.3})
    local function rel(w) return {w[1],w[2]-lc,w[3]-0.040} end -- P-relative offset -> pulley-relative
    cyl('Rep5x_CFlangeA',0.045,0.0015,cLink,rel({0,lc,0.040-0.00675}),COL.alu,'z',SPEC)
    cyl('Rep5x_CFlangeB',0.045,0.0015,cLink,rel({0,lc,0.040+0.00675}),COL.alu,'z',SPEC)
    cyl('Rep5x_SlipRingHolder',0.018,0.014,cLink,rel({0,lc,0.053}),COL.printed,'z')
    cyl('Rep5x_SlipRing',0.012,0.010,cLink,rel({0,lc,0.065}),COL.motor,'z')
    local inlet=sim.createDummy(0.004)
    setAlias(inlet,'Rep5x_BowdenInlet')
    child(inlet,cLink,rel({0,lc,0.072}))
    -- B arm (printed): top plate under the pulley, two cheeks, B axle through P.
    box('Rep5x_BArmPlate',{0.050,0.044,0.006},cLink,rel({0,0,0.031}),COL.printed)
    box('Rep5x_BArmCheekA',{0.040,0.006,0.040},cLink,rel({0,-0.019,0.010}),COL.printed)
    box('Rep5x_BArmCheekB',{0.040,0.006,0.040},cLink,rel({0,0.019,0.010}),COL.printed)
    cyl('Rep5x_BAxle',0.006,0.052,cLink,rel({0,0,0}),COL.alu,'y',SPEC)
    cyl('Rep5x_BowdenUpper',0.004,0.024,cLink,rel({0,lc,0.016}),COL.ptfe,'z')
    -- B stepper (NEMA14) outside the +Y cheek, belt down to the B driven pulley.
    local bm={0,0.040,0.045}
    stepper('Rep5x_BStepper',cLink,rel(bm),0.035,0.026,{0,-1,0},0.0)
    cyl('Rep5x_BMotorPulley',0.012,0.010,cLink,rel({0,0.0295,bm[3]}),COL.alu,'y',SPEC)
    for i,s in ipairs({-1,1}) do
        member('Rep5x_BBelt'..i,cLink,rel({s*0.0055,0.0295,bm[3]}),rel({s*0.0200,0.0295,0.0}),0.006,COL.belt,{kind='slab',w=0.0015,hint={0,1,0}})
    end

    -- A revolute joint rotates around local Z. Rx(-90deg) aligns it to +Y. Placed at P.
    local b=joint(profile.joints.B,sim.joint_revolute,cLink,rel({0,0,0}),{-math.pi/2,0,0},false,{profile.limits.B[1],profile.limits.B[2]-profile.limits.B[1]})
    -- B link alias = hotend holder block, re-aligned to the world frame. In the B
    -- frame local +Y maps to world -Z at B=0, so world (x,y,z) -> (x,-z,y).
    local holderZ=-0.012
    local bLink=box(profile.objects.bLink,{0.030,0.030,0.016},b,{0,-holderZ,0},COL.printed,{math.pi/2,0,0})
    -- Children of bLink are world aligned at B=0; coordinates relative to P.
    local function brel(w) return {w[1],w[2],w[3]-holderZ} end
    -- Hotend: heatsink with fins, cooling fan, heat break, silicone-socked block.
    cyl('Rep5x_Heatsink',0.022,0.022,bLink,brel({0,0,-0.025}),COL.alu,'z',SPEC)
    for i,z in ipairs({-0.016,-0.0195,-0.023,-0.0265,-0.030,-0.0335}) do
        cyl('Rep5x_HeatsinkFin'..i,0.026,0.0012,bLink,brel({0,0,z}),COL.alu,'z',SPEC)
    end
    box('Rep5x_HotendFan',{0.030,0.010,0.030},bLink,brel({0,-0.0205,-0.025}),COL.fan)
    cyl('Rep5x_HotendFanHub',0.012,0.002,bLink,brel({0,-0.0265,-0.025}),COL.motorFace,'y')
    cyl('Rep5x_HeatBreak',0.0045,0.0027,bLink,brel({0,0,-0.03735}),COL.alu,'z',SPEC)
    box('Rep5x_HeaterBlock',{0.023,0.016,0.0115},bLink,brel({0,0,-0.04442}),COL.sock)
    member('Rep5x_HeaterWires',bLink,brel({0.010,0,-0.0445}),brel({0.022,0.012,-0.006}),0.003,COL.wire)
    -- B driven pulley on the +Y side of the arm.
    cyl('Rep5x_BDrivenPulley',0.041,0.012,bLink,brel({0,0.0295,0}),COL.steel,'y',{0.3,0.3,0.3})
    cyl('Rep5x_BFlangeA',0.045,0.0015,bLink,brel({0,0.0295-0.00675,0}),COL.alu,'y',SPEC)
    cyl('Rep5x_BFlangeB',0.045,0.0015,bLink,brel({0,0.0295+0.00675,0}),COL.alu,'y',SPEC)
    -- Brass nozzle: cone apex points down to the tip at (0,0,-LB).
    local nozzleH=0.00447
    local nozzle=prim(profile.objects.nozzle,sim.primitiveshape_cone,{0.008,0.008,nozzleH},bLink,brel({0,0,-profile.lb+nozzleH/2}),{math.pi,0,0},COL.brass,SPEC)
    -- Molten bead at the tip; shown only while extruding.
    local drop=sphere('Rep5x_MeltDrop',0.0012,bLink,brel({0,0,-profile.lb}),COL.filament)
    pcall(sim.setObjectInt32Param,drop,sim.objintparam_visibility_layer,0)
    local tip=sim.createDummy(0.004)
    setAlias(tip,profile.objects.tip)
    child(tip,bLink,brel({0,0,-profile.lb}))
    return c,cLink,b,bLink,nozzle,tip
end

local function buildScene()
    if findExact('/'..profile.root) then error('scene already contains /'..profile.root..'; refusing to create duplicate handles') end
    local root=sim.createDummy(0.01)
    setAlias(root,profile.root)
    state.handles.root=root
    buildFrame(root)
    buildBed(root)
    local _,_,x=buildGantry(root)
    local carriage,crel=buildCarriage(x)
    buildHead(carriage,crel)
    pcall(sim.setBoolProperty,root,'modelBase',true)
    validateHandles()
    state.pose={X=0,Y=0,Z=0,C=0,B=0}
    state.extrusion={handle=nil,points=0,filamentMm=0,perItemSize=false,full=false,first=nil,last=nil}
    status('scene ready; Ender-3 V3 SE / Rep5x proxy geometry (kinematic, non-dynamic)')
end

-- ---------------------------------------------------------------------------
-- Kinematics and pose application
-- ---------------------------------------------------------------------------
local function ik(p)
    local c=math.rad(p.C)
    local b=math.rad(p.B)
    return {
        X=p.X/1000-math.sin(c)*profile.lc+math.cos(c)*math.sin(b)*profile.lb,
        Y=p.Y/1000+(math.cos(c)-1)*profile.lc+math.sin(c)*math.sin(b)*profile.lb,
        Z=p.Z/1000+(math.cos(b)-1)*profile.lb,
        C=c,
        B=b,
    }
end

local function checkPose(p,line)
    local values={p.X,p.Y,p.Z,p.C,p.B}
    for _,v in ipairs(values) do if v~=v or v==math.huge or v==-math.huge then error('NaN/Infinity at line '..tostring(line)) end end
    if p.X<0 or p.X>200 then error('X soft limit at line '..tostring(line)) end
    if p.Y< -40 or p.Y>200 then error('Y soft limit at line '..tostring(line)) end
    if p.Z<0 or p.Z>174.6 then error('Z soft limit at line '..tostring(line)) end
    if p.B< -135 or p.B>135 then error('B soft limit at line '..tostring(line)) end
end

local function applyPose(p,line)
    checkPose(p,line)
    local q=ik(p)
    if q.X<profile.limits.X[1] or q.X>profile.limits.X[2] then error('IK X joint soft limit at line '..tostring(line)) end
    if q.Y<profile.limits.Y[1] or q.Y>profile.limits.Y[2] then error('IK Y joint soft limit at line '..tostring(line)) end
    if q.Z<profile.limits.Z[1] or q.Z>profile.limits.Z[2] then error('IK Z joint soft limit at line '..tostring(line)) end
    if q.B<profile.limits.B[1] or q.B>profile.limits.B[2] then error('IK B joint soft limit at line '..tostring(line)) end
    -- One callback/tick owns all five writes; jog and playback cannot race.
    sim.setJointPosition(state.handles[profile.joints.X],q.X*profile.signs.X)
    sim.setJointPosition(state.handles[profile.joints.Y],q.Y*profile.signs.Y)
    sim.setJointPosition(state.handles[profile.joints.Z],q.Z*profile.signs.Z)
    sim.setJointPosition(state.handles[profile.joints.C],q.C*profile.signs.C)
    sim.setJointPosition(state.handles[profile.joints.B],q.B*profile.signs.B)
    state.pose={X=p.X,Y=p.Y,Z=p.Z,C=p.C,B=p.B}
end

-- ---------------------------------------------------------------------------
-- G-code parsing (G0/G1/G28/G92, G20/G21/G70/G71, G90/G91, M82/M83, E or A)
-- ---------------------------------------------------------------------------
local function strip(line)
    return line:gsub('%b()',''):gsub(';.*$',''):match('^%s*(.-)%s*$')
end

local IGNORED_M={[82]=true,[83]=true,[84]=true,[104]=true,[105]=true,[106]=true,[107]=true,[109]=true,[140]=true,[190]=true,[17]=true,[18]=true,[73]=true,[117]=true,[201]=true,[203]=true,[204]=true,[205]=true,[220]=true,[221]=true,[900]=true}

local function parseGcode(path)
    local f,err=io.open(path,'r')
    if not f then error('cannot open G-code: '..tostring(err)) end
    local moves,warnings={},{}
    local p={X=0,Y=0,Z=0,C=0,B=0}
    local absolute,scale=true,1
    local eAbs,ePos,feed=true,0,nil
    local hasBC,hasMeta,hasE=false,false,false
    local lineNo=0
    local skippedM=0
    for raw in f:lines() do
        lineNo=lineNo+1
        if raw:upper():find('CONICAL_META',1,true) then hasMeta=true end
        local code=strip(raw)
        if code~='' then
            local words={}
            for letter,value in code:gmatch('([%a])%s*([%+%-]?[%d%.]+)') do words[letter:upper()]=tonumber(value) end
            if words.G==90 then absolute=true elseif words.G==91 then absolute=false elseif words.G==20 or words.G==70 then scale=25.4 elseif words.G==21 or words.G==71 then scale=1 end
            if words.M==82 then eAbs=true elseif words.M==83 then eAbs=false
            elseif words.M and not IGNORED_M[words.M] then skippedM=skippedM+1; if skippedM<=5 then table.insert(warnings,'line '..lineNo..': skipped M'..words.M) end end
            if words.F then feed=words.F*scale end
            if words.G==0 or words.G==1 then
                local n={X=p.X,Y=p.Y,Z=p.Z,C=p.C,B=p.B,line=lineNo,rapid=(words.G==0),feed=feed,extrudeMm=0,extrude=false}
                for _,axis in ipairs({'X','Y','Z','C','B'}) do
                    if words[axis] then
                        local value=words[axis]*((axis=='X' or axis=='Y' or axis=='Z') and scale or 1)
                        n[axis]=absolute and value or p[axis]+value
                        if axis=='C' or axis=='B' then hasBC=true end
                    end
                end
                local ev=words.E or words.A
                if ev then
                    hasE=true
                    local delta
                    if eAbs then delta=ev-ePos; ePos=ev else delta=ev; ePos=ePos+ev end
                    n.extrudeMm=delta
                    n.extrude=(words.G==1 and delta>0)
                end
                checkPose(n,lineNo)
                table.insert(moves,n)
                p=n
            elseif words.G==28 then
                -- Home: listed axes (or all) go to zero as a rapid travel.
                local n={X=p.X,Y=p.Y,Z=p.Z,C=p.C,B=p.B,line=lineNo,rapid=true,feed=feed,extrudeMm=0,extrude=false}
                local any=false
                for _,axis in ipairs({'X','Y','Z','C','B'}) do if words[axis] then n[axis]=0; any=true end end
                if not any then n.X,n.Y,n.Z,n.C,n.B=0,0,0,0,0 end
                table.insert(moves,n)
                p=n
            elseif words.G==92 then
                for _,axis in ipairs({'X','Y','Z','C','B'}) do
                    if words[axis] then p[axis]=words[axis]*((axis=='X' or axis=='Y' or axis=='Z') and scale or 1) end
                end
                local ev=words.E or words.A
                if ev then ePos=ev end
                checkPose(p,lineNo)
            elseif words.G and words.G~=20 and words.G~=21 and words.G~=70 and words.G~=71 and words.G~=90 and words.G~=91 and words.G~=92 then
                error('unsupported G-code G'..words.G..' at line '..lineNo..': '..raw)
            end
        end
    end
    f:close()
    if skippedM>5 then table.insert(warnings,(skippedM-5)..' more M-codes skipped') end
    if #moves==0 then error('trajectory queue is empty') end
    if not hasBC then table.insert(warnings,'XYZ-only compatibility mode: B=C=0/modal') end
    if hasMeta then table.insert(warnings,'CONICAL_META ignored; no B/C inference in phase 1') end
    if not hasE then table.insert(warnings,'no E/A words: nothing will be extruded') end
    return moves,(hasBC and '5-axis' or 'XYZ-only compatibility mode'),warnings
end

-- ---------------------------------------------------------------------------
-- Filament deposition. Bead samples are parented to the DrawBoard (G-code
-- origin on the moving bed), so printed material travels with the bed and
-- sample coordinates equal the commanded tool XYZ in metres.
-- ---------------------------------------------------------------------------
local function ensureExtrusionObject()
    local e=state.extrusion
    if e.handle then return e.handle end
    local board=state.handles[profile.objects.drawBoard]
    if not board then return nil end
    local flags=sim.drawing_spherepoints
    if sim.drawing_persistent then flags=flags+sim.drawing_persistent end
    e.perItemSize=false
    if sim.drawing_itemsizes then
        local ok,h=pcall(sim.addDrawingObject,flags+sim.drawing_itemsizes,EXT.beadWidthMm/1000,0,board,EXT.maxPoints,EXT.color)
        if ok and h and h>=0 then e.handle=h; e.perItemSize=true; return h end
    end
    local ok,h=pcall(sim.addDrawingObject,flags,EXT.beadWidthMm/1000,0,board,EXT.maxPoints,EXT.color)
    if ok and h and h>=0 then e.handle=h; return h end
    sim.addLog(sim.verbosity_scripterrors,'[Rep5x] cannot create extrusion drawing object: '..tostring(h))
    e.handle=false
    return nil
end

local function clearExtrusion()
    local e=state.extrusion
    if e.handle then pcall(sim.addDrawingObjectItem,e.handle,nil) end
    e.points,e.filamentMm,e.full,e.first,e.last=0,0,false,nil,nil
end

local function beadWidthM(seg)
    local m=seg.move
    if seg.len<=1e-9 or m.extrudeMm<=0 then return EXT.beadWidthMm/1000 end
    local area=m.extrudeMm*math.pi*(EXT.filamentDiameterMm/2)^2/seg.len -- mm^2 of bead cross-section
    local width=1.4*2*math.sqrt(area/math.pi)
    width=math.max(EXT.minBeadMm,math.min(EXT.maxBeadMm,width))
    return width/1000
end

-- Items given to a parented drawing object are world coordinates at the time of
-- the call; CoppeliaSim stores them relative to the parent, so beads follow the bed.
local function bedToWorld(x,y,z)
    local m=sim.getObjectMatrix(state.handles[profile.objects.drawBoard],-1)
    return {m[1]*x+m[2]*y+m[3]*z+m[4],m[5]*x+m[6]*y+m[7]*z+m[8],m[9]*x+m[10]*y+m[11]*z+m[12]}
end

local function deposit(x,y,z,widthM)
    local e=state.extrusion
    if e.full then return end
    local h=ensureExtrusionObject()
    if not h then return end
    if e.points>=EXT.maxPoints then
        e.full=true
        status(string.format('extrusion buffer full at %d bead samples; increase EXT.maxPoints',EXT.maxPoints))
        return
    end
    local item=bedToWorld(x/1000,y/1000,z/1000)
    if e.perItemSize then item[4]=widthM end
    local ok,err=pcall(sim.addDrawingObjectItem,h,item)
    if not ok then
        sim.addLog(sim.verbosity_scripterrors,'[Rep5x] extrusion sample rejected: '..tostring(err))
        e.handle=nil
        return
    end
    e.points=e.points+1
    e.last={x,y,z}
    if not e.first then e.first={x,y,z} end
end

local function setMeltVisible(visible)
    if state.meltVisible==visible or not state.meltDrop then return end
    state.meltVisible=visible
    pcall(sim.setObjectInt32Param,state.meltDrop,sim.objintparam_visibility_layer,visible and 1 or 0)
end

-- Dynamic Bowden tube between the frame-mounted extruder and the head.
local function updateBowden()
    local bw=state.bowden
    if bw.disabled or not bw.outlet or not bw.inlet then return end
    local ok,err=pcall(function()
        if not bw.handle then
            local kind=sim.drawing_linestrip or sim.drawing_lines
            local flags=kind
            if sim.drawing_persistent then flags=flags+sim.drawing_persistent end
            bw.handle=sim.addDrawingObject(flags,4,0,-1,64,COL.ptfe)
            bw.strip=(sim.drawing_linestrip~=nil)
        end
        local a=sim.getObjectPosition(bw.outlet,-1)
        local b=sim.getObjectPosition(bw.inlet,-1)
        local ctrl={(a[1]+b[1])/2,(a[2]+b[2])/2,math.max(a[3],b[3])+0.12}
        sim.addDrawingObjectItem(bw.handle,nil)
        local n=14
        local prev=nil
        for i=0,n do
            local t=i/n
            local u=1-t
            local pt={u*u*a[1]+2*u*t*ctrl[1]+t*t*b[1],u*u*a[2]+2*u*t*ctrl[2]+t*t*b[2],u*u*a[3]+2*u*t*ctrl[3]+t*t*b[3]}
            if bw.strip then
                sim.addDrawingObjectItem(bw.handle,pt)
            elseif prev then
                sim.addDrawingObjectItem(bw.handle,{prev[1],prev[2],prev[3],pt[1],pt[2],pt[3]})
            end
            prev=pt
        end
    end)
    if not ok then
        bw.disabled=true
        sim.addLog(sim.verbosity_scriptwarnings,'[Rep5x] Bowden tube visual disabled: '..tostring(err))
    end
end

-- ---------------------------------------------------------------------------
-- Interpolated playback
-- ---------------------------------------------------------------------------
local function lerpPose(a,b,u)
    return {X=a.X+(b.X-a.X)*u,Y=a.Y+(b.Y-a.Y)*u,Z=a.Z+(b.Z-a.Z)*u,C=a.C+(b.C-a.C)*u,B=a.B+(b.B-a.B)*u}
end

local function startSegment(move)
    local from=state.pose
    local dx,dy,dz=move.X-from.X,move.Y-from.Y,move.Z-from.Z
    local len=math.sqrt(dx*dx+dy*dy+dz*dz)
    local rot=math.max(math.abs(move.C-from.C),math.abs(move.B-from.B))
    local feed=move.feed or EXT.defaultFeedMmMin
    if move.rapid then feed=math.max(feed,EXT.rapidFeedMmMin) end
    local duration=math.max(len/(feed/60),rot/EXT.rotaryDegPerS,1e-6)
    local seg={move=move,from={X=from.X,Y=from.Y,Z=from.Z,C=from.C,B=from.B},len=len,duration=duration,t=0,lastDot=0,dotted=false}
    seg.width=beadWidthM(seg)
    state.seg=seg
    return seg
end

local function stepSegment(dt)
    local seg=state.seg
    seg.t=seg.t+dt
    local u=math.min(1,seg.t/seg.duration)
    local pose=lerpPose(seg.from,seg.move,u)
    if seg.move.extrude then
        local spacing=EXT.dotSpacingMm
        local s=u*seg.len
        if seg.len<=1e-9 then
            if not seg.dotted then deposit(pose.X,pose.Y,pose.Z,seg.width); seg.dotted=true end
        else
            if not seg.dotted then deposit(seg.from.X,seg.from.Y,seg.from.Z,seg.width); seg.dotted=true end
            while seg.lastDot+spacing<=s do
                seg.lastDot=seg.lastDot+spacing
                local q=lerpPose(seg.from,seg.move,seg.lastDot/seg.len)
                deposit(q.X,q.Y,q.Z,seg.width)
            end
            if u>=1 and seg.lastDot<seg.len then deposit(seg.move.X,seg.move.Y,seg.move.Z,seg.width) end
        end
        if u>=1 then state.extrusion.filamentMm=state.extrusion.filamentMm+seg.move.extrudeMm end
    end
    setMeltVisible(seg.move.extrude)
    applyPose(pose,seg.move.line)
    return u>=1
end

local function verifyTipAgainstCommand()
    if state.mismatchWarned then return end
    local tip,board=state.handles[profile.objects.tip],state.handles[profile.objects.drawBoard]
    if not tip or not board then return end
    local r=sim.getObjectPosition(tip,board)
    local p=state.pose
    local err=math.max(math.abs(r[1]*1000-p.X),math.abs(r[2]*1000-p.Y),math.abs(r[3]*1000-p.Z))
    if err>0.05 then
        state.mismatchWarned=true
        sim.addLog(sim.verbosity_scriptwarnings,string.format('[Rep5x] nozzle tip differs from commanded pose by %.3f mm; scene geometry and profile disagree',err))
    end
end

local function refresh()
    if not state.ui then return end
    local p=state.pose
    simUI.setLabelText(state.ui,202,string.format('X %.3f  Y %.3f  Z %.3f mm   C %.3f  B %.3f deg',p.X,p.Y,p.Z,p.C,p.B))
    local tip=state.handles[profile.objects.tip]
    if tip then
        local xyz=sim.getObjectPosition(tip,-1)
        simUI.setLabelText(state.ui,203,string.format('nozzle world: %.3f, %.3f, %.3f mm',xyz[1]*1000,xyz[2]*1000,xyz[3]*1000))
    end
    local e=state.extrusion
    simUI.setLabelText(state.ui,204,string.format('filament %.1f mm | %d bead samples%s',e.filamentMm,e.points,state.meltVisible and ' | extruding' or ''))
end

function rep5xBuild()
    local ok,err=pcall(buildScene)
    if not ok then fail(err) end
end

local function loadGcodeFile(path)
    local ok,moves,mode,warnings=pcall(parseGcode,path)
    if not ok then fail(moves); return false end
    state.moves,state.index,state.running,state.paused,state.seg=moves,1,false,false,nil
    local extruding=0
    for _,m in ipairs(moves) do if m.extrude then extruding=extruding+1 end end
    status(mode..'; loaded '..#moves..' moves ('..extruding..' extruding); '..table.concat(warnings,' | '))
    return true
end

function rep5xOpen()
    local result=simUI.fileDialog(simUI.filedialog_type.load,'Open G-code','','','G-code','gcode;gc;nc',true)
    if #result==0 then return end
    loadGcodeFile(result[1])
end

function rep5xStart()
    if #state.moves==0 then fail('trajectory queue is empty; open a G-code file or load the test trajectory'); return end
    if not state.handles[profile.joints.X] then fail('scene not connected; build scene first'); return end
    state.index=1; state.running=true; state.paused=false; state.seg=nil
    status('running '..state.mode)
end

function rep5xPause() if state.running then state.paused=true; status('paused') end end
function rep5xResume() if state.running then state.paused=false; status('running') end end
function rep5xStop() state.running=false; state.paused=false; state.index=1; state.seg=nil; setMeltVisible(false); status('stopped') end
function rep5xClearPrint() clearExtrusion(); status('printed material cleared') end

function rep5xHome()
    if state.running and not state.paused then fail('pause trajectory before home/jog'); return end
    local ok,err=pcall(applyPose,{X=0,Y=0,Z=0,C=0,B=0},0)
    if not ok then fail(err) else status('home pose') end
end

function rep5xJog(_,id)
    if state.running and not state.paused then fail('jog rejected while trajectory is running; pause first'); return end
    local map={[301]={'X',-1},[302]={'X',1},[303]={'Y',-1},[304]={'Y',1},[305]={'Z',-1},[306]={'Z',1},[307]={'C',-1},[308]={'C',1},[309]={'B',-1},[310]={'B',1}}
    local item=map[id]
    local p={X=state.pose.X,Y=state.pose.Y,Z=state.pose.Z,C=state.pose.C,B=state.pose.B}
    local step=5
    p[item[1]]=p[item[1]]+item[2]*step
    local ok,err=pcall(applyPose,p,0)
    if not ok then fail(err) else status('manual jog') end
end

function rep5xSpeed(_,_,index)
    local speeds={0.1,0.5,1,2,5}
    state.speed=speeds[index+1] or 1
    status(string.format('speed %.1fx',state.speed))
end

function rep5xTest()
    -- A first-layer square is deposited, then the phase-1 five-axis sweep runs in the air.
    local function mv(x,y,z,c,b,e,rapid) return {X=x,Y=y,Z=z,C=c,B=b,line=0,rapid=rapid or false,feed=1200,extrudeMm=e or 0,extrude=(e or 0)>0} end
    state.moves={
        mv(20,20,0.2,0,0,0,true),
        mv(80,20,0.2,0,0,2.0),mv(80,80,0.2,0,0,2.0),mv(20,80,0.2,0,0,2.0),mv(20,20,0.2,0,0,2.0),
        mv(25,25,0.2,0,0,0,true),
        mv(75,25,0.2,0,0,1.66),mv(75,75,0.2,0,0,1.66),mv(25,75,0.2,0,0,1.66),mv(25,25,0.2,0,0,1.66),
        mv(30,30,100,0,0,0,true),
        mv(80,30,100,0,0),mv(80,80,100,0,0),mv(80,80,140,0,0),
        mv(80,80,140,90,0),mv(80,80,140,90,45),
        mv(100,90,130,135,-35),mv(80,80,120,0,0),
    }
    state.index=1; state.running=false; state.paused=false; state.seg=nil
    status('built-in test loaded: first-layer squares + 5-axis sweep')
end

function sysCall_init()
    state.selfTestPath=sim.getNamedStringParam('rep5x.selfTest')
    if state.selfTestPath then
        local ok,err=pcall(buildScene)
        if not ok then fail(err); return end
        local gcodePath=sim.getNamedStringParam('rep5x.gcode')
        if gcodePath and gcodePath~='' then
            if not loadGcodeFile(gcodePath) then return end
        else
            rep5xTest()
        end
        status('headless self-test ready')
        return
    end
    local xml=[[
    <ui title="Rep5x control" closeable="false" resizable="true" layout="vbox" placement="relative" position="20,20">
      <group layout="hbox"><button text="Build scene" on-click="rep5xBuild"/><button text="Open G-code" on-click="rep5xOpen"/><button text="Test print" on-click="rep5xTest"/></group>
      <group layout="hbox"><button text="Start" on-click="rep5xStart"/><button text="Pause" on-click="rep5xPause"/><button text="Resume" on-click="rep5xResume"/><button text="Stop" on-click="rep5xStop"/><button text="Home" on-click="rep5xHome"/><button text="Clear print" on-click="rep5xClearPrint"/></group>
      <group layout="form"><label text="Speed"/><combobox on-change="rep5xSpeed"><item>0.1x</item><item>0.5x</item><item selected="true">1x</item><item>2x</item><item>5x</item></combobox></group>
      <group layout="grid">
        <label text="X"/><button id="301" text="-" on-click="rep5xJog"/><button id="302" text="+" on-click="rep5xJog"/>
        <label text="Y"/><button id="303" text="-" on-click="rep5xJog"/><button id="304" text="+" on-click="rep5xJog"/>
        <label text="Z"/><button id="305" text="-" on-click="rep5xJog"/><button id="306" text="+" on-click="rep5xJog"/>
        <label text="C"/><button id="307" text="-" on-click="rep5xJog"/><button id="308" text="+" on-click="rep5xJog"/>
        <label text="B"/><button id="309" text="-" on-click="rep5xJog"/><button id="310" text="+" on-click="rep5xJog"/>
      </group>
      <label id="201" text="plugin: not probed; scene: not built"/>
      <label id="202" text="X 0 Y 0 Z 0 mm C 0 B 0 deg"/>
      <label id="203" text="nozzle world: unavailable"/>
      <label id="204" text="filament 0.0 mm | 0 bead samples"/>
    </ui>]]
    state.ui=simUI.create(xml)
    if findExact('/'..profile.root) then
        local ok,err=pcall(validateHandles)
        if ok then status('scene connected; plugin not yet probed') else fail(err) end
    end
end

function sysCall_beforeSimulation()
    if not state.handles[profile.joints.X] then return end

    -- Presentation/local mode:
    -- Do not load the legacy simExtFIBR3D C++ plugin.
    status('local G-code mode; plugin disabled')

    if state.selfTestPath then
        rep5xStart()
    end
end

function sysCall_actuation()
    if state.running and not state.paused then
        local dt=sim.getSimulationTimeStep()*state.speed
        if not state.seg then
            local move=state.moves[state.index]
            if not move then
                state.running=false
                setMeltVisible(false)
                status(string.format('trajectory complete; %d bead samples, %.1f mm filament',state.extrusion.points,state.extrusion.filamentMm))
                writeSelfTest(true,'trajectory complete')
                return
            end
            startSegment(move)
        end
        local ok,done=pcall(stepSegment,dt)
        if not ok then fail(done); return end
        verifyTipAgainstCommand()
        if done then
            state.index=state.index+1
            state.seg=nil
        end
    end
end

function sysCall_sensing()
    refresh()
    updateBowden()
end

function sysCall_cleanup()
    if state.selfTestPath and not state.selfTestWritten then writeSelfTest(false,'simulation ended before trajectory completion') end
    if state.ui then simUI.destroy(state.ui); state.ui=nil end
end
