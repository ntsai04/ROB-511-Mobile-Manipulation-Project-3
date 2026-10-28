"use strict";
const $=id=>document.getElementById(id),canvas=$("scene"),ctx=canvas.getContext("2d");
let model=null,worlds=new Map(),yaw=-.8,pitch=.55,zoom=1,center=[0,0,0],scale=170;
const direct=(el,name)=>Array.from(el.children).find(e=>e.localName===name);
function vector(text){const values=text.trim().split(/[,\s]+/).map(Number);if(values.length!==3||values.some(v=>!Number.isFinite(v)))throw Error("Enter three finite numbers for each global pose field.");return values;}
function readRobot(){
  const xml=new DOMParser().parseFromString($("urdf").value,"application/xml");
  if(xml.querySelector("parsererror"))throw Error("The robot description must be valid XML.");
  const robot=xml.documentElement;
  if(robot.localName!=="robot")throw Error("The description must contain a robot element.");
  const links=Array.from(robot.children).filter(e=>e.localName==="link").map(e=>e.getAttribute("name"));
  const joints=Array.from(robot.children).filter(e=>e.localName==="joint").map(el=>({name:el.getAttribute("name"),type:el.getAttribute("type"),parent:direct(el,"parent")?.getAttribute("link"),child:direct(el,"child")?.getAttribute("link")}));
  return {name:robot.getAttribute("name")||"Robot",links,joints};
}
function rebuild(){
  const values=jointValues();model=readRobot();$("robot-name").textContent=model.name;
  $("joints").replaceChildren();
  for(const j of model.joints){
    if(j.type==="fixed")continue;
    const label=document.createElement("label"),name=document.createElement("span"),type=document.createElement("small"),input=document.createElement("input");
    label.className="joint-label";name.textContent=j.name;type.textContent=j.type==="prismatic"?"meters":"radians";label.append(name,type);
    input.type="number";input.step="0.05";input.value=values[j.name]??(j.type==="prismatic"?.25:.6);input.dataset.joint=j.name;input.setAttribute("aria-label",j.name+" position");
    $("joints").append(label,input);
  }
  if(!$("joints").children.length){const text=document.createElement("p");text.textContent="This robot has no movable joints.";$("joints").append(text);}
}
function jointValues(){return Object.fromEntries(Array.from($("joints").querySelectorAll("input")).map(el=>[el.dataset.joint,Number(el.value)]));}
function fit(){
  const points=[...worlds.values()].map(m=>m.slice(12,15)).filter(p=>p.every(Number.isFinite));
  if(!points.length)return;
  const lo=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),hi=[0,1,2].map(i=>Math.max(...points.map(p=>p[i])));
  center=lo.map((v,i)=>(v+hi[i])/2);
  const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
  const projected=points.map(p=>{const [x,y,z]=p.map((v,i)=>v-center[i]);return [cy*x-sy*y,sp*(sy*x+cy*y)-cp*z];});
  const span=[0,1].map(i=>Math.max(...projected.map(p=>p[i]))-Math.min(...projected.map(p=>p[i])));
  scale=Math.min(800/Math.max(.6,span[0]),500/Math.max(.6,span[1]));zoom=1;
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
  }else{ctx.fillStyle="#7f9aaf";ctx.font="18px system-ui";ctx.textAlign="center";ctx.fillText("Select Show pose to display the robot",540,340);ctx.textAlign="left";}
  requestAnimationFrame(draw);
}
async function apply(){
  const button=$("apply");button.disabled=true;$("notice").className="";$("notice").textContent="Loading robot pose…";
  try{
    rebuild();const joints=jointValues(),xyz=vector($("xyz").value),rpy=vector($("rpy").value);
    if(Object.values(joints).some(v=>!Number.isFinite(v)))throw Error("Joint positions must be finite numbers.");
    const response=await fetch("/api/evaluate",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({urdf:$("urdf").value,joints,xyz,rpy})});
    const data=await response.json();if(!response.ok||data.error)throw Error(data.error||"Unable to load this robot pose.");
    worlds=new Map(data.world.transforms.map(e=>[e.child_frame_id,e.matrix]));
    $("robot-name").textContent=model.name;
    $("notice").textContent=worlds.size?"Pose displayed.":"No robot pose is available for this configuration.";
    fit();
  }catch(error){showError(error);}finally{button.disabled=false;}
}
async function example(){const response=await fetch("/api/example");if(!response.ok)throw Error("Unable to load the example robot.");$("urdf").value=(await response.json()).urdf;rebuild();worlds.clear();$("notice").textContent="Example loaded. Select Show pose to display it.";}
function showError(error){$("notice").className="error";$("notice").textContent=error.message;}
$("apply").onclick=apply;$("sample").onclick=()=>example().catch(showError);$("parse").onclick=()=>{try{rebuild();$("notice").className="";$("notice").textContent="Joint controls updated.";}catch(error){showError(error);}};$("fit").onclick=fit;
let drag=null;canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};canvas.onpointerup=()=>drag=null;canvas.onpointercancel=()=>drag=null;canvas.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.4,Math.min(1.4,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];};canvas.addEventListener("wheel",e=>{e.preventDefault();zoom=Math.max(.1,Math.min(10,zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
example().catch(showError);requestAnimationFrame(draw);
