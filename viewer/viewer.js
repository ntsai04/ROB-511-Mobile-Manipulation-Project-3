"use strict";
const $=id=>document.getElementById(id),canvas=$("scene"),ctx=canvas.getContext("2d");
const DEG=180/Math.PI,UNBOUNDED=new Set(["continuous"]),MOVABLE=new Set(["revolute","continuous","prismatic"]);
let model=null,description="",worlds=new Map(),yaw=-.8,pitch=.55,zoom=1,center=[0,0,0],scale=170;
let epoch=0,mode="ready",eventSource=null,motionId=null,currentConfig=null,currentDescription=null,needsFit=true;
let operations=Promise.resolve(),sequence=null,pauseRequested=false;
const direct=(el,name)=>Array.from(el.children).find(e=>e.localName===name);
const pretty=value=>Number(value.toPrecision(10)).toString();
const clone=value=>JSON.parse(JSON.stringify(value));
const displayValue=(joint,value)=>joint.type==="prismatic"?value:value*DEG;
const unit=joint=>joint.type==="prismatic"?"m":"°";
function readRobot(urdf){
  const xml=new DOMParser().parseFromString(urdf,"application/xml");
  if(xml.querySelector("parsererror"))throw Error("The robot description must be valid XML.");
  const robot=xml.documentElement;if(robot.localName!=="robot")throw Error("The description must contain a robot element.");
  const links=Array.from(robot.children).filter(e=>e.localName==="link").map(e=>e.getAttribute("name"));
  const names=new Set();
  const joints=Array.from(robot.children).filter(e=>e.localName==="joint").map(el=>{
    const name=el.getAttribute("name"),type=el.getAttribute("type");
    if(!name||!name.trim()||names.has(name))throw Error("Each joint needs a unique name.");names.add(name);
    if(type!=="fixed"&&!MOVABLE.has(type))throw Error("Unsupported joint type for "+name+": "+type+".");
    const limit=direct(el,"limit");let lower=null,upper=null;
    if(!UNBOUNDED.has(type)&&type!=="fixed"){
      if(!limit)throw Error("Joint "+name+" needs a limit element in the URDF.");
      const lowerText=limit.getAttribute("lower")??"0",upperText=limit.getAttribute("upper")??"0";
      if(!lowerText.trim()||!upperText.trim())throw Error("Invalid URDF limits for "+name+".");
      lower=Number(lowerText);upper=Number(upperText);
      if(!Number.isFinite(lower)||!Number.isFinite(upper)||lower>upper)throw Error("Invalid URDF limits for "+name+".");
    }
    return {name,type,parent:direct(el,"parent")?.getAttribute("link"),child:direct(el,"child")?.getAttribute("link"),lower,upper};
  });
  return {name:robot.getAttribute("name")||"Robot",links,joints};
}
function homeConfiguration(){return {joints:Object.fromEntries(model.joints.filter(j=>MOVABLE.has(j.type)).map(j=>[j.name,Math.max(j.lower??-Infinity,Math.min(j.upper??Infinity,0))])),xyz:[0,0,0],rpy:[0,0,0]};}
function fillTargets(config){for(const j of model.joints){const input=Array.from($("joints").querySelectorAll("input")).find(el=>el.dataset.joint===j.name);if(input){input.value=pretty(displayValue(j,config.joints[j.name]??0));input.setCustomValidity("");}}}
function rebuild(){
  $("robot-name").textContent=model.name;$("joint-count").textContent=model.joints.filter(j=>MOVABLE.has(j.type)).length+" movable";$("joints").replaceChildren();
  const fixed=model.joints.filter(j=>j.type==="fixed"),fixedGroup=document.createElement("details"),fixedSummary=document.createElement("summary");
  fixedGroup.className="fixed-joints";fixedSummary.textContent=fixed.length+" fixed joints";fixedGroup.append(fixedSummary);
  for(const [index,j] of model.joints.entries()){
    const row=document.createElement("div"),top=document.createElement("div"),label=document.createElement("label"),type=document.createElement("span");
    row.className="joint-row"+(j.type==="fixed"?" fixed-row":"");top.className="joint-top";label.textContent=j.name;type.className="joint-type";type.textContent=j.type;top.append(label,type);row.append(top);
    if(j.type==="fixed"){const text=document.createElement("span");text.className="fixed-value";text.textContent="Fixed · no motion input";row.append(text);}
    else{
      const input=document.createElement("input"),field=document.createElement("div"),suffix=document.createElement("span"),limits=document.createElement("span");
      input.type="number";input.step="any";input.required=true;input.id="joint-"+index;input.dataset.joint=j.name;label.htmlFor=input.id;input.setAttribute("aria-label",j.name+" target ("+(j.type==="prismatic"?"metres":"degrees")+")");
      if(j.lower!==null)input.min=pretty(displayValue(j,j.lower));if(j.upper!==null)input.max=pretty(displayValue(j,j.upper));
      limits.id="limit-"+index;limits.className="joint-limits";input.setAttribute("aria-describedby",limits.id);
      limits.textContent=j.type==="continuous"?"Continuous rotation · no angle limits":j.lower===null?"No position limits specified in this URDF":"URDF limits: "+pretty(displayValue(j,j.lower))+" to "+pretty(displayValue(j,j.upper))+" "+unit(j);
      input.addEventListener("input",()=>input.setCustomValidity(""));field.className="joint-input";suffix.className="unit";suffix.textContent=unit(j);field.append(input,suffix);row.append(field,limits);
    }
    (j.type==="fixed"?fixedGroup:$("joints")).append(row);
  }
  if(fixed.length)$("joints").append(fixedGroup);
  if(!model.joints.length){const p=document.createElement("p");p.textContent="This robot has no joints.";p.className="explanation";$("joints").append(p);}
  fillTargets(homeConfiguration());
}
function target(){
  const result=homeConfiguration();
  for(const j of model.joints.filter(j=>MOVABLE.has(j.type))){
    const input=Array.from($("joints").querySelectorAll("input")).find(el=>el.dataset.joint===j.name),value=Number(input.value);
    const lower=j.lower===null?null:displayValue(j,j.lower),upper=j.upper===null?null:displayValue(j,j.upper),epsilon=1e-8*Math.max(1,Math.abs(value));
    let error="";if(input.value.trim()===""||!Number.isFinite(value))error="Enter a finite position for "+j.name+".";
    else if(lower!==null&&(value<lower-epsilon||value>upper+epsilon))error=j.name+" must be between "+pretty(lower)+" and "+pretty(upper)+" "+unit(j)+".";
    input.setCustomValidity(error);if(error){input.reportValidity();throw Error(error);}
    const position=j.type==="prismatic"?value:value/DEG;result.joints[j.name]=Math.max(j.lower??-Infinity,Math.min(j.upper??Infinity,position));
  }
  return result;
}
function duration(){const value=Number($("duration").value);if(!Number.isFinite(value)||value<.2||value>120)throw Error("Use a duration from 0.2 to 120 seconds.");return value;}
function controls(){
  const available=!!model,active=!!sequence||!!eventSource||mode==="loading";
  for(const id of ["play","choreography","reset"])$(id).disabled=!available||mode==="loading";
  $("pause").disabled=!sequence||pauseRequested;$("resume").disabled=!sequence||!pauseRequested;$("stop").disabled=!active;
  $("playback-mode").textContent={ready:"READY",loading:"LOADING",playing:"PLAYING",pausing:"PAUSING",paused:"PAUSED",complete:"COMPLETE",stopped:"STOPPED"}[mode]||"READY";
}
function notice(text,error=false){$("notice").className=error?"error":"";if($("notice").textContent!==text)$("notice").textContent=text;}
function serial(fn){const result=operations.catch(()=>{}).then(fn);operations=result.catch(()=>{});return result;}
async function post(path,body={}){const response=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});const data=await response.json();if(!response.ok||data.error)throw Error(data.error||"The robot request could not be completed.");return data;}
function detach(){epoch++;sequence=null;pauseRequested=false;if(eventSource){eventSource.close();eventSource=null;}const old=motionId;motionId=null;if(old)serial(()=>post('/api/motion/'+old+'/stop').catch(()=>{}));}
function stop(text="Playback stopped."){detach();mode="stopped";controls();notice(text);}
function fail(error){stop();notice(error.message,true);}
function showWorld(data,fitOnce=false){
  if(!data?.world||!Array.isArray(data.world.transforms))throw Error("The robot pose is unavailable.");
  worlds=new Map(data.world.transforms.map(e=>[e.child_frame_id,e.matrix]));
  if(data.configuration)currentConfig=data.configuration;
  if(fitOnce)fit();
}
function choreography(){
  const home=homeConfiguration(),goals=[];
  const phases=[-.65,.65,.2,-.35,0];
  for(const [index,phase] of phases.entries()){
    const config=clone(home);
    for(const [i,j] of model.joints.filter(j=>MOVABLE.has(j.type)).entries()){
      if(index===phases.length-1){config.joints[j.name]=home.joints[j.name];continue;}
      const offset=i%2===0?phase:-phase;
      if(j.lower!==null)config.joints[j.name]=(j.lower/2+j.upper/2)+offset*(j.upper/2-j.lower/2);
      else config.joints[j.name]=offset*(j.type==="prismatic"?.15:Math.PI*.75);
    }
    goals.push(config);
  }
  return goals;
}
async function begin(goals,kind){
  let seconds;try{seconds=duration();}catch(error){notice(error.message,true);return;}
  stop("Starting playback…");const token=epoch;sequence={goals:goals.map(clone),index:0,kind,seconds,token};needsFit=worlds.size===0;mode="loading";controls();
  await startSegment(token);
}
async function startSegment(token){
  if(token!==epoch||!sequence)return;
  const run=sequence,goal=run.goals[run.index],known=currentDescription===description?currentConfig:null,start=known?clone(known):homeConfiguration();
  mode="loading";controls();
  try{
    const result=await serial(async()=>{
      if(token!==epoch)return null;
      const value=await post('/api/motion',{urdf:description,start,goal,duration:run.seconds});
      if(token!==epoch){await post('/api/motion/'+value.id+'/stop').catch(()=>{});return null;}return value;
    });
    if(token!==epoch||!result||sequence!==run)return;motionId=result.id;currentDescription=description;
    if(pauseRequested)await serial(()=>post('/api/motion/'+result.id+'/pause'));
    if(token!==epoch||sequence!==run)return;
    const current=new EventSource('/api/motion/'+result.id+'/events');eventSource=current;mode=pauseRequested?"paused":"playing";controls();
    current.onmessage=event=>{
      if(token!==epoch||eventSource!==current||sequence!==run)return;
      try{
        const data=JSON.parse(event.data);showWorld(data,needsFit);needsFit=false;
        const segment=run.kind==="choreography"?" · movement "+(run.index+1)+"/"+run.goals.length:"";
        $("clock").textContent=data.elapsed.toFixed(2)+" / "+data.duration.toFixed(2)+" s"+segment;
        $("progress").style.width=(100*data.elapsed/data.duration)+"%";
        if(!pauseRequested){mode="playing";notice(run.kind==="choreography"?"Playing choreography · movement "+(run.index+1)+" of "+run.goals.length+". Repeats until stopped.":"Moving to the entered configuration.");}
        controls();
      }catch(error){fail(error);}
    };
    current.addEventListener('playback',event=>{if(token!==epoch||eventSource!==current)return;try{if(JSON.parse(event.data).state==='paused'&&pauseRequested){mode='paused';controls();notice('Playback paused.');}}catch(error){fail(error);}});
    current.addEventListener('done',event=>{
      if(token!==epoch||eventSource!==current||sequence!==run)return;
      try{
        const data=JSON.parse(event.data);current.close();eventSource=null;motionId=null;
        if(data.state!=='complete'){stop();return;}
        if(run.kind==="choreography"){
          run.index=(run.index+1)%run.goals.length;startSegment(token);
        }else{sequence=null;pauseRequested=false;mode='complete';controls();notice('Playback complete.');}
      }catch(error){fail(error);}
    });
    current.addEventListener('stream_error',event=>{if(token!==epoch||eventSource!==current)return;try{fail(Error(JSON.parse(event.data).error));}catch(error){fail(error);}});
    current.onerror=()=>{if(token===epoch&&eventSource===current)fail(Error('The pose stream stopped. Start make run in another terminal and try again.'));};
  }catch(error){if(token===epoch)fail(error);}
}
async function command(action){
  if(!sequence)return;
  const token=epoch,id=motionId;pauseRequested=action==='pause';mode=pauseRequested?'pausing':'playing';controls();notice(pauseRequested?'Pausing playback…':'Playback resumed.');
  if(!id){mode=pauseRequested?'paused':'loading';controls();return;}
  try{
    const result=await serial(()=>token===epoch&&id===motionId?post('/api/motion/'+id+'/'+action):null);
    if(token!==epoch||id!==motionId||!sequence||!result)return;
    if(result.state==='paused'&&pauseRequested)mode='paused';else if(result.state==='playing'&&!pauseRequested)mode='playing';controls();
  }catch(error){if(token===epoch&&id===motionId&&sequence)fail(error);}
}
async function loadRobot(urdf,source){
  let parsed;try{parsed=readRobot(urdf);}catch(error){notice(error.message,true);return;}
  stop("Loading robot…");const token=epoch;model=parsed;description=urdf;currentConfig=null;currentDescription=null;worlds.clear();needsFit=true;$("urdf").value=urdf;$("robot-source").textContent=source;rebuild();mode="loading";controls();$("clock").textContent="0.00 s";$("progress").style.width="0%";
  try{
    const data=await serial(()=>token===epoch?post('/api/evaluate',{urdf,...homeConfiguration()}):null);
    if(token!==epoch||!data)return;showWorld(data,true);needsFit=false;currentDescription=urdf;mode="ready";controls();notice("Enter a joint configuration, or select Play choreography.");
  }catch(error){if(token===epoch)fail(error);}
}
async function example(){
  stop("Loading default arm…");const token=epoch;
  try{const response=await fetch('/api/example');if(!response.ok)throw Error('Unable to load the default arm.');const data=await response.json();if(token===epoch)await loadRobot(data.urdf,"Default arm");}
  catch(error){if(token===epoch)fail(error);}
}
function fit(){
  const points=[...worlds.values()].map(m=>m.slice(12,15)).filter(p=>p.every(Number.isFinite));if(!points.length)return;
  const lo=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),hi=[0,1,2].map(i=>Math.max(...points.map(p=>p[i])));center=lo.map((v,i)=>(v+hi[i])/2);
  const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),projected=points.map(p=>{const [x,y,z]=p.map((v,i)=>v-center[i]);return [cy*x-sy*y,sp*(sy*x+cy*y)-cp*z];});
  const span=[0,1].map(i=>Math.max(...projected.map(p=>p[i]))-Math.min(...projected.map(p=>p[i])));scale=Math.min(650/Math.max(.9,span[0]),420/Math.max(.9,span[1]));zoom=1;
}
function project(p){const [x,y,z]=p.map((v,i)=>v-center[i]),cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),u=cy*x-sy*y,v=sy*x+cy*y;return [540+u*scale*zoom,350+(sp*v-cp*z)*scale*zoom];}
function line(a,b,color,width=1){const p=project(a),q=project(b);ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
function draw(){
  ctx.fillStyle="#0e1b28";ctx.fillRect(0,0,1080,700);
  for(let i=-10;i<=10;i++){line([i*.25,-2.5,0],[i*.25,2.5,0],"#223749");line([-2.5,i*.25,0],[2.5,i*.25,0],"#223749");}
  if(model&&worlds.size){
    for(const j of model.joints){const p=worlds.get(j.parent),c=worlds.get(j.child);if(p&&c&&p.every(Number.isFinite)&&c.every(Number.isFinite))line(p.slice(12,15),c.slice(12,15),"#7fe4d3",6);}
    for(const [name,m] of worlds){
      if(m.length!==16||!m.every(Number.isFinite))continue;const p=m.slice(12,15);
      if($("show-axes").checked)for(let axis=0;axis<3;axis++)line(p,p.map((v,i)=>v+m[4*axis+i]*.12),["#ef827c","#80d2a6","#7cb4ed"][axis],2);
      const s=project(p);ctx.beginPath();ctx.arc(...s,4,0,Math.PI*2);ctx.fillStyle="#d6f5f0";ctx.fill();
      if($("show-labels").checked){ctx.fillStyle="#d3e7f2";ctx.font="12px system-ui";ctx.fillText(name,s[0]+8,s[1]-8);}
    }
  }else{ctx.fillStyle="#7f9aaf";ctx.font="18px system-ui";ctx.textAlign="center";ctx.fillText("Start make run to display the loaded robot",540,340);ctx.textAlign="left";}
  requestAnimationFrame(draw);
}
let drag=null;
canvas.addEventListener("pointerdown",event=>{drag={x:event.clientX,y:event.clientY};canvas.setPointerCapture(event.pointerId);});
canvas.addEventListener("pointermove",event=>{if(!drag)return;yaw+=(event.clientX-drag.x)*.008;pitch=Math.max(-1.5,Math.min(1.5,pitch+(event.clientY-drag.y)*.008));drag={x:event.clientX,y:event.clientY};});
for(const name of ["pointerup","pointercancel","lostpointercapture"])canvas.addEventListener(name,()=>drag=null);
canvas.addEventListener("wheel",event=>{event.preventDefault();zoom=Math.max(.15,Math.min(8,zoom*Math.exp(-event.deltaY*.001)));},{passive:false});
$("configuration-form").noValidate=true;$("configuration-form").onsubmit=event=>{event.preventDefault();try{begin([target()],"manual");}catch(error){notice(error.message,true);}};
$("choreography").onclick=()=>begin(choreography(),"choreography");$("reset").onclick=()=>{const config=homeConfiguration();fillTargets(config);begin([config],"home");};
$("pause").onclick=()=>command('pause');$("resume").onclick=()=>command('resume');$("stop").onclick=()=>stop();$("fit").onclick=fit;$("sample").onclick=example;
$("parse").onclick=()=>loadRobot($("urdf").value,"Custom URDF");
$("urdf-file").onchange=async event=>{
  const input=event.target,file=input.files[0];if(!file)return;
  stop("Reading URDF file…");const token=epoch;mode="loading";controls();
  try{if(file.size>2_000_000)throw Error("Choose a URDF smaller than 2 MB.");const text=await file.text();if(token===epoch){mode="stopped";controls();await loadRobot(text,file.name);}}
  catch(error){if(token===epoch){mode="stopped";controls();notice(error.message,true);}}
  if(input.files[0]===file)input.value="";
};
window.addEventListener('pagehide',()=>{const id=motionId;epoch++;if(eventSource)eventSource.close();if(id)fetch('/api/motion/'+id+'/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',keepalive:true}).catch(()=>{});});
controls();example();requestAnimationFrame(draw);
