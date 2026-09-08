-- Rep5x phase-1 scene builder and kinematic controller.
-- No serial, USB, firmware, or real-printer API is used by this add-on.

local sim=require('sim')
local simUI=nil
if not sim.getBoolParam(sim.boolparam_headless) then simUI=require('simUI') end
-- CoppeliaSim 4.10 loads installed add-ons from an in-memory string, so
-- debug.getinfo() no longer exposes the source file's directory.
local scriptDir=sim.getStringParam(sim.stringparam_application_path)..'/addOns'
local profile=dofile(scriptDir..'/rep5x_profile.lua')

local state={ui=nil,handles={},moves={},index=1,running=false,paused=false,speed=1,lastStep=0,pose={X=0,Y=0,Z=0,C=0,B=0},mode='idle',selfTestPath=nil,selfTestWritten=false}

local function jsonString(value)
    return '"'..tostring(value):gsub('\\','\\\\'):gsub('"','\\"'):gsub('\r','\\r'):gsub('\n','\\n')..'"'
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
    local payload=string.format(
        '{\n  "ok": %s,\n  "message": %s,\n  "coppelia_version": %s,\n  "moves_completed": %d,\n  "final_pose_mm_deg": {"X": %.9g, "Y": %.9g, "Z": %.9g, "C": %.9g, "B": %.9g},\n  "scene_joints_m_rad": {"X": %.12g, "Y": %.12g, "Z": %.12g, "C": %.12g, "B": %.12g},\n  "nozzle_world_mm": [%.12g, %.12g, %.12g],\n  "tcp_relative_bed_mm": [%.12g, %.12g, %.12g]\n}\n',
        tostring(ok),jsonString(message),jsonString(sim.getInt32Param(sim.intparam_program_full_version)),math.max(0,state.index-1),
        p.X,p.Y,p.Z,p.C,p.B,joints.X,joints.Y,joints.Z,joints.C,joints.B,tip[1]*1000,tip[2]*1000,tip[3]*1000,tipBed[1]*1000,tipBed[2]*1000,tipBed[3]*1000)
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
    status('ERROR: '..text)
    sim.addLog(sim.verbosity_scripterrors,'[Rep5x] '..text)
    writeSelfTest(false,text)
end

local function setAlias(handle,name)
    sim.setObjectAlias(handle,name)
end

local function child(handle,parent,pos,ori)
    sim.setObjectParent(handle,parent,false)
    sim.setObjectPosition(handle,parent,pos or {0,0,0})
    sim.setObjectOrientation(handle,parent,ori or {0,0,0})
    return handle
end

local function shape(name,size,parent,pos,color)
    local h=sim.createPrimitiveShape(sim.primitiveshape_cuboid,size,0)
    setAlias(h,name)
    child(h,parent,pos)
    sim.setObjectInt32Param(h,sim.shapeintparam_static,1)
    sim.setShapeColor(h,nil,sim.colorcomponent_ambient_diffuse,color or {0.45,0.48,0.52})
    return h
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
end

local function buildScene()
    if findExact('/'..profile.root) then error('scene already contains /'..profile.root..'; refusing to create duplicate handles') end
    local root=sim.createDummy(0.01)
    setAlias(root,profile.root)
    state.handles.root=root

    local frame=sim.createDummy(0.01)
    setAlias(frame,profile.objects.frame)
    child(frame,root)
    shape('Rep5x_FrameBase',{0.33,0.05,0.04},frame,{0,0,0.02},{0.18,0.20,0.23})
    shape('Rep5x_FrameLeft',{0.04,0.04,0.37},frame,{-0.145,0,0.205},{0.18,0.20,0.23})
    shape('Rep5x_FrameRight',{0.04,0.04,0.37},frame,{0.145,0,0.205},{0.18,0.20,0.23})
    shape('Rep5x_FrameTop',{0.33,0.04,0.04},frame,{0,0,0.39},{0.18,0.20,0.23})

    local y=joint(profile.joints.Y,sim.joint_prismatic,root,{0,0,0},{-math.pi/2,0,0},false,{-profile.limits.Y[2],profile.limits.Y[2]-profile.limits.Y[1]})
    local bed=shape(profile.objects.bed,{0.22,0.22,0.005},y,{0,0.003,0},{0.20,0.25,0.30})
    sim.setObjectOrientation(bed,y,{math.pi/2,0,0})
    shape(profile.objects.drawBoard,{0.20,0.20,0.001},bed,{0,0,0.003},{0.08,0.10,0.12})

    local z=joint(profile.joints.Z,sim.joint_prismatic,root,{0,0,0},{0,0,0},false,{profile.limits.Z[1],profile.limits.Z[2]-profile.limits.Z[1]})
    local gantry=shape(profile.objects.gantry,{0.30,0.035,0.035},z,{0,0,0},{0.35,0.38,0.42})
    local x=joint(profile.joints.X,sim.joint_prismatic,gantry,{0,0,0},{0,math.pi/2,0},false,{profile.limits.X[1],profile.limits.X[2]-profile.limits.X[1]})
    local carriage=shape(profile.objects.carriage,{0.055,0.045,0.065},x,{0,0,0},{0.85,0.35,0.10})
    sim.setObjectOrientation(carriage,x,{0,-math.pi/2,0})

    -- C pivot is +LC in Y and +LB in Z from the zero-pose TCP.
    local c=joint(profile.joints.C,sim.joint_revolute,carriage,{0,profile.lc,profile.lb},{0,0,0},true,{0,2*math.pi})
    local cLink=shape(profile.objects.cLink,{0.045,0.045,0.025},c,{0,-profile.lc,0},{0.20,0.55,0.85})
    -- A revolute joint rotates around local Z. Rx(-90deg) aligns it to +Y.
    local b=joint(profile.joints.B,sim.joint_revolute,cLink,{0,0,0},{-math.pi/2,0,0},false,{profile.limits.B[1],profile.limits.B[2]-profile.limits.B[1]})
    local bLink=shape(profile.objects.bLink,{0.05,0.025,0.025},b,{0,0,0},{0.30,0.75,0.45})
    local nozzle=sim.createPrimitiveShape(sim.primitiveshape_cone,{0.012,0.012,0.035},0)
    setAlias(nozzle,profile.objects.nozzle)
    -- In the B-joint frame local +Y maps to world -Z at B=0.
    child(nozzle,bLink,{0,profile.lb-0.0175,0},{math.pi/2,0,0})
    sim.setObjectInt32Param(nozzle,sim.shapeintparam_static,1)
    sim.setShapeColor(nozzle,nil,sim.colorcomponent_ambient_diffuse,{0.75,0.58,0.20})
    local tip=sim.createDummy(0.004)
    setAlias(tip,profile.objects.tip)
    child(tip,bLink,{0,profile.lb,0})

    validateHandles()
    state.pose={X=0,Y=0,Z=0,C=0,B=0}
    status('scene ready; proxy geometry (kinematic, non-dynamic)')
end

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

local function strip(line)
    return line:gsub('%b()',''):gsub(';.*$',''):match('^%s*(.-)%s*$')
end

local function parseGcode(path)
    local f,err=io.open(path,'r')
    if not f then error('cannot open G-code: '..tostring(err)) end
    local moves,warnings={},{}
    local p={X=0,Y=0,Z=0,C=0,B=0}
    local absolute,scale=true,1
    local hasBC,hasMeta=false,false
    local lineNo=0
    for raw in f:lines() do
        lineNo=lineNo+1
        if raw:upper():find('CONICAL_META',1,true) then hasMeta=true end
        local code=strip(raw)
        if code~='' then
            local words={}
            for letter,value in code:gmatch('([%a])%s*([%+%-]?[%d%.]+)') do words[letter:upper()]=tonumber(value) end
            if words.G==90 then absolute=true elseif words.G==91 then absolute=false elseif words.G==20 or words.G==70 then scale=25.4 elseif words.G==21 or words.G==71 then scale=1 end
            if words.M then table.insert(warnings,'line '..lineNo..': skipped M'..words.M) end
            if words.G==0 or words.G==1 then
                local n={X=p.X,Y=p.Y,Z=p.Z,C=p.C,B=p.B,line=lineNo}
                for _,axis in ipairs({'X','Y','Z','C','B'}) do
                    if words[axis] then
                        local value=words[axis]*((axis=='X' or axis=='Y' or axis=='Z') and scale or 1)
                        n[axis]=absolute and value or p[axis]+value
                        if axis=='C' or axis=='B' then hasBC=true end
                    end
                end
                checkPose(n,lineNo)
                table.insert(moves,n)
                p=n
            elseif words.G==92 then
                for _,axis in ipairs({'X','Y','Z','C','B'}) do
                    if words[axis] then p[axis]=words[axis]*((axis=='X' or axis=='Y' or axis=='Z') and scale or 1) end
                end
                checkPose(p,lineNo)
            elseif words.G and words.G~=20 and words.G~=21 and words.G~=70 and words.G~=71 and words.G~=90 and words.G~=91 and words.G~=92 then
                error('unsupported G-code G'..words.G..' at line '..lineNo..': '..raw)
            end
        end
    end
    f:close()
    if #moves==0 then error('trajectory queue is empty') end
    if not hasBC then table.insert(warnings,'XYZ-only compatibility mode: B=C=0/modal') end
    if hasMeta then table.insert(warnings,'CONICAL_META ignored; no B/C inference in phase 1') end
    return moves,(hasBC and '5-axis' or 'XYZ-only compatibility mode'),warnings
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
end

function rep5xBuild()
    local ok,err=pcall(buildScene)
    if not ok then fail(err) end
end

function rep5xOpen()
    local result=simUI.fileDialog(simUI.filedialog_type.load,'Open G-code','','','G-code','gcode;gc;nc',true)
    if #result==0 then return end
    local ok,moves,mode,warnings=pcall(parseGcode,result[1])
    if not ok then fail(moves); return end
    state.moves,state.index,state.running,state.paused=moves,1,false,false
    status(mode..'; loaded '..#moves..' moves; '..table.concat(warnings,' | '))
end

function rep5xStart()
    if #state.moves==0 then fail('trajectory queue is empty; open a G-code file or load the test trajectory'); return end
    if not state.handles[profile.joints.X] then fail('scene not connected; build scene first'); return end
    state.index=1; state.running=true; state.paused=false; state.lastStep=sim.getSimulationTime()
    status('running '..state.mode)
end

function rep5xPause() if state.running then state.paused=true; status('paused') end end
function rep5xResume() if state.running then state.paused=false; state.lastStep=sim.getSimulationTime(); status('running') end end
function rep5xStop() state.running=false; state.paused=false; state.index=1; status('stopped') end

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
    local step=(item[1]=='C' or item[1]=='B') and 5 or 5
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
    state.moves={
        {X=30,Y=30,Z=100,C=0,B=0,line=0},{X=80,Y=30,Z=100,C=0,B=0,line=0},
        {X=80,Y=80,Z=100,C=0,B=0,line=0},{X=80,Y=80,Z=140,C=0,B=0,line=0},
        {X=80,Y=80,Z=140,C=90,B=0,line=0},{X=80,Y=80,Z=140,C=90,B=45,line=0},
        {X=100,Y=90,Z=130,C=135,B=-35,line=0},{X=80,Y=80,Z=120,C=0,B=0,line=0},
    }
    state.index=1; state.running=false; state.paused=false
    status('built-in 5-axis test trajectory loaded')
end

function sysCall_init()
    state.selfTestPath=sim.getNamedStringParam('rep5x.selfTest')
    if state.selfTestPath then
        local ok,err=pcall(buildScene)
        if not ok then fail(err); return end
        rep5xTest()
        status('headless self-test ready')
        return
    end
    local xml=[[
    <ui title="Rep5x phase-1 control" closeable="false" resizable="true" layout="vbox" placement="relative" position="20,20">
      <group layout="hbox"><button text="Build scene" on-click="rep5xBuild"/><button text="Open G-code" on-click="rep5xOpen"/><button text="5-axis test" on-click="rep5xTest"/></group>
      <group layout="hbox"><button text="Start" on-click="rep5xStart"/><button text="Pause" on-click="rep5xPause"/><button text="Resume" on-click="rep5xResume"/><button text="Stop" on-click="rep5xStop"/><button text="Home" on-click="rep5xHome"/></group>
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
        local now=sim.getSimulationTime()
        if now-state.lastStep>=0.15/state.speed then
            local move=state.moves[state.index]
            if not move then state.running=false; status('trajectory complete'); writeSelfTest(true,'trajectory complete'); return end
            local ok,err=pcall(applyPose,move,move.line)
            if not ok then fail(err); return end
            state.index=state.index+1
            state.lastStep=now
        end
    end
end

function sysCall_sensing() refresh() end
function sysCall_cleanup()
    if state.selfTestPath and not state.selfTestWritten then writeSelfTest(false,'simulation ended before trajectory completion') end
    if state.ui then simUI.destroy(state.ui); state.ui=nil end
end
