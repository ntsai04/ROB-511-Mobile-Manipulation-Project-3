"use strict";
const $=id=>document.getElementById(id),canvas=$("scene"),ctx=canvas.getContext("2d");
let model=null,worlds=new Map(),yaw=-.8,pitch=.55,zoom=1,center=[0,0,0],scale=170;
let epoch=0,mode="stopped",eventSource=null,motionId=null,currentConfig=null,currentDescription=null,needsFit=true;
let operations=Promise.resolve();
const direct=(el,name)=>Array.from(el.children).find(e=>e.localName===name);
function vector(text){const values=text.trim().split(/[,\s]+/).map(Number);if(values.length!==3||values.some(v=>!Number.isFinite(v)))throw Error("Enter three finite numbers for each global pose field.");return values;}
function readRobot(){
  const xml=new DOMParser().parseFromString($("urdf").value,"application/xml");
  if(xml.querySelector("parsererror"))throw Error("The robot description must be valid XML.");
  const robot=xml.documentElement;if(robot.localName!=="robot")throw Error("The description must contain a robot element.");
  const links=Array.from(robot.children).filter(e=>e.localName==="link").map(e=>e.getAttribute("name"));
  const joints=Array.from(robot.children).filter(e=>e.localName==="joint").map(el=>({name:el.getAttribute("name"),type:el.getAttribute("type"),parent:direct(el,"parent")?.getAttribute("link"),child:direct(el,"child")?.getAttribute("link")}));
  return {name:robot.getAttribute("name")||"Robot",links,joints};
}
function jointValues(){return Object.fromEntries(Array.from($("joints").querySelectorAll("input")).map(el=>[el.dataset.joint,Number(el.value)]));}
function rebuild(){
  const values=jointValues();model=readRobot();$("robot-name").textContent=model.name;$("joints").replaceChildren();
  for(const j of model.joints){
    if(j.type==="fixed")continue;
    const label=document.createElement("label"),name=document.createElement("span"),type=document.createElement("small"),input=document.createElement("input");
    label.className="joint-label";name.textContent=j.name;type.textContent=j.type==="prismatic"?"meters":"radians";label.append(name,type);
    input.type="number";input.step="0.05";input.value=values[j.name]??(j.type==="prismatic"?.45:1.2);input.dataset.joint=j.name;input.setAttribute("aria-label",j.name+" target position");input.addEventListener("input",inputsChanged);$("joints").append(label,input);
  }
  if(!$("joints").children.length){const text=document.createElement("p");text.textContent="This robot has no movable joints.";$("joints").append(text);}
}
function target(){
  rebuild();const result={joints:jointValues(),xyz:vector($("xyz").value),rpy:vector($("rpy").value)};
  if(Object.values(result.joints).some(v=>!Number.isFinite(v)))throw Error("Joint targets must be finite numbers.");return result;
}
function controls(){
  $("play").disabled=mode==="loading";$("apply").disabled=mode==="loading";
  $("pause").disabled=mode!=="playing";$("resume").disabled=mode!=="paused";
  $("stop").disabled=!eventSource&&mode!=="loading";$("watch").disabled=mode==="watching";
  $("playback-mode").textContent={stopped:"STOPPED",loading:"LOADING",playing:"PLAYING",pausing:"PAUSING",paused:"PAUSED",complete:"COMPLETE",watching:"LIVE RUNTIME"}[mode];
}
function notice(text,error=false){$("notice").className=error?"error":"";if($("notice").textContent!==text)$("notice").textContent=text;}
function serial(fn){const result=operations.catch(()=>{}).then(fn);operations=result.catch(()=>{});return result;}
async function post(path,body={}){const response=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});const data=await response.json();if(!response.ok||data.error)throw Error(data.error||"The robot request could not be completed.");return data;}
function detach(){epoch++;if(eventSource){eventSource.close();eventSource=null;}const old=motionId;motionId=null;if(old)serial(()=>post('/api/motion/'+old+'/stop').catch(()=>{}));}
function stop(text="Playback stopped. The last received pose remains displayed."){detach();mode="stopped";controls();notice(text);}
function fail(error){stop();notice(error.message,true);}
function inputsChanged(event){
  if(event?.target?.id==="urdf"){currentConfig=null;currentDescription=null;}
  if(eventSource||mode==="loading")stop("Targets changed. Select Play to target to start a new motion.");
}
function showWorld(data,fitOnce=false){
  if(!data?.world||!Array.isArray(data.world.transforms))throw Error("The robot pose is unavailable.");
  worlds=new Map(data.world.transforms.map(e=>[e.child_frame_id,e.matrix]));
  if(data.configuration)currentConfig=data.configuration;
  if(fitOnce)fit();
}
async function apply(){
  stop("Loading target pose…");const token=epoch;mode="loading";controls();
  try{const goal=target(),urdf=$("urdf").value;const data=await serial(()=>token===epoch?post('/api/evaluate',{urdf,...goal}):null);if(token!==epoch||!data)return;showWorld(data,true);currentDescription=urdf;mode="stopped";controls();notice(worlds.size?"Target pose displayed.":"No robot pose is available for this configuration.");}
  catch(error){if(token===epoch)fail(error);}
}
async function play(){
  stop("Starting motion…");const token=epoch;mode="loading";controls();
  try{
    const goal=target(),urdf=$("urdf").value,duration=Number($("duration").value);
    if(!Number.isFinite(duration)||duration<.2||duration>120)throw Error("Use a duration from 0.2 to 120 seconds.");
    const known=currentDescription===urdf?currentConfig:null;
    const start={joints:Object.fromEntries(Object.keys(goal.joints).map(name=>[name,known?.joints?.[name]??0])),xyz:known?.xyz||[0,0,0],rpy:known?.rpy||[0,0,0]};
    const result=await serial(async()=>{const value=await post('/api/motion',{urdf,start,goal,duration});if(token!==epoch){await post('/api/motion/'+value.id+'/stop').catch(()=>{});return null;}return value;});
    if(token!==epoch||!result)return;motionId=result.id;currentDescription=urdf;needsFit=true;$("clock").textContent='0.00 / '+duration.toFixed(2)+' s';
    const current=new EventSource('/api/motion/'+motionId+'/events');eventSource=current;
    current.onmessage=event=>{if(token!==epoch||eventSource!==current)return;try{const data=JSON.parse(event.data);showWorld(data,needsFit);needsFit=false;$("clock").textContent=data.elapsed.toFixed(2)+' / '+data.duration.toFixed(2)+' s';if(mode!=="pausing")mode="playing";controls();notice("Playing the entered joint and global-pose targets.");}catch(error){fail(error);}};
    current.addEventListener('playback',event=>{if(token!==epoch||eventSource!==current)return;try{const data=JSON.parse(event.data);if(data.state==='paused'){mode='paused';controls();notice('Playback paused. Select Resume to continue.');}}catch(error){fail(error);}});
    current.addEventListener('done',event=>{if(token!==epoch||eventSource!==current)return;try{const data=JSON.parse(event.data);current.close();eventSource=null;motionId=null;mode=data.state==='complete'?'complete':'stopped';controls();notice(data.state==='complete'?'Playback complete.':'Playback stopped.');}catch(error){fail(error);}});
    current.addEventListener('stream_error',event=>{if(token!==epoch||eventSource!==current)return;try{fail(Error(JSON.parse(event.data).error));}catch(error){fail(error);}});
    current.onerror=()=>{if(token===epoch&&eventSource===current)fail(Error('The pose stream stopped. Start the project runtime and try again.'));};
  }catch(error){if(token===epoch)fail(error);}
}
async function command(action){
  if(!motionId)return;const token=epoch,id=motionId;
  try{if(action==='pause'){mode='pausing';controls();notice('Pausing playback…');}const result=await serial(()=>post('/api/motion/'+id+'/'+action));if(token!==epoch||id!==motionId||!eventSource)return;if(action==='resume'&&result.state==='playing'){mode='playing';controls();notice('Playback resumed.');}}
  catch(error){if(token===epoch&&id===motionId&&eventSource)fail(error);}
}
function watch(){
  stop("Connecting to the running robot…");const token=epoch,current=new EventSource('/api/stream');eventSource=current;mode='watching';needsFit=true;controls();
  current.addEventListener('robot',event=>{if(token!==epoch||eventSource!==current)return;try{const value=JSON.parse(event.data);if(currentDescription!==value.urdf)currentConfig=null;$("urdf").value=value.urdf;rebuild();currentDescription=value.urdf;needsFit=true;}catch(error){fail(error);}});
  current.onmessage=event=>{if(token!==epoch||eventSource!==current)return;try{const data=JSON.parse(event.data);showWorld(data,needsFit);needsFit=false;const stamp=data.world.transforms[0]?.header?.stamp;$("clock").textContent=stamp?(Number(stamp.sec)+Number(stamp.nanosec)/1e9).toFixed(2)+' s':'—';notice('Following poses published by the running project.');}catch(error){fail(error);}};
  current.addEventListener('stream_error',event=>{if(token===epoch&&eventSource===current){try{fail(Error(JSON.parse(event.data).error));}catch(error){fail(error);}}});
  current.onerror=()=>{if(token===epoch&&eventSource===current)fail(Error('The pose stream stopped. Start the project runtime and try again.'));};
}
async function example(){
  stop("Loading example…");const token=epoch;
  try{const response=await fetch('/api/example');if(!response.ok)throw Error('Unable to load the example robot.');const data=await response.json();if(token!==epoch)return;$("urdf").value=data.urdf;$("joints").replaceChildren();rebuild();currentConfig=null;currentDescription=null;worlds.clear();notice('Example loaded. Play to target starts from zero joint positions.');}
  catch(error){if(token===epoch)fail(error);}
}
function fit(){
  const points=[...worlds.values()].map(m=>m.slice(12,15)).filter(p=>p.every(Number.isFinite));
  if(!points.length)return;
  const lo=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),hi=[0,1,2].map(i=>Math.max(...points.map(p=>p[i])));
  center=lo.map((v,i)=>(v+hi[i])/2);
  const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
  const projected=points.map(p=>{const [x,y,z]=p.map((v,i)=>v-center[i]);return [cy*x-sy*y,sp*(sy*x+cy*y)-cp*z];});
  const span=[0,1].map(i=>Math.max(...projected.map(p=>p[i]))-Math.min(...projected.map(p=>p[i])));
  scale=Math.min(650/Math.max(.9,span[0]),420/Math.max(.9,span[1]));zoom=1;
}
function project(p){const [x,y,z]=p.map((v,i)=>v-center[i]),cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),u=cy*x-sy*y,v=sy*x+cy*y;return [540+u*scale*zoom,350+(sp*v-cp*z)*scale*zoom];}
function line(a,b,color,width=1){const p=project(a),q=project(b);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
function draw(){
  ctx.fillStyle="#0e1b28";ctx.fillRect(0,0,1080,700);
  for(let i=-10;i<=10;i++){line([i*.25,-2.5,0],[i*.25,2.5,0],"#223749");line([-2.5,i*.25,0],[2.5,i*.25,0],"#223749");}
  if(model&&worlds.size){
    for(const j of model.joints){const p=worlds.get(j.parent),c=worlds.get(j.child);if(p&&c&&p.every(Number.isFinite)&&c.every(Number.isFinite))line(p.slice(12,15),c.slice(12,15),"#7fe4d3",5);}
    for(const [name,m] of worlds){
      if(m.length!==16||!m.every(Number.isFinite))continue;
      const p=m.slice(12,15);
      for(let axis=0;axis<3;axis++)line(p,p.map((v,i)=>v+m[4*axis+i]*.14),["#ef827c","#80d2a6","#7cb4ed"][axis],2);
      const s=project(p);ctx.beginPath();ctx.arc(...s,4,0,Math.PI*2);ctx.fillStyle="#d6f5f0";ctx.fill();ctx.fillStyle="#d3e7f2";ctx.font="12px system-ui";ctx.fillText(name,s[0]+8,s[1]-8);
    }
  }else{ctx.fillStyle="#7f9aaf";ctx.font="18px system-ui";ctx.textAlign="center";ctx.fillText("Select Play to target or Show target pose",540,340);ctx.textAlign="left";}
  requestAnimationFrame(draw);
}

$("apply").onclick=apply;$("play").onclick=play;$("pause").onclick=()=>command('pause');$("resume").onclick=()=>command('resume');$("stop").onclick=()=>stop();$("watch").onclick=watch;$("fit").onclick=fit;$("sample").onclick=example;
$("parse").onclick=()=>{stop('Joint target controls updated.');try{rebuild();}catch(error){fail(error);}};
for(const id of ['urdf','xyz','rpy','duration'])$(id).addEventListener('input',inputsChanged);
window.addEventListener('pagehide',()=>{const id=motionId;epoch++;if(eventSource)eventSource.close();if(id)fetch('/api/motion/'+id+'/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',keepalive:true}).catch(()=>{});});

controls();example();requestAnimationFrame(draw);
