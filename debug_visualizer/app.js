'use strict';
const $ = id => document.getElementById(id);
const sample = `<robot name="frame_lab">
  <link name="base"/><link name="shoulder"/><link name="elbow"/><link name="tool"/>
  <joint name="yaw" type="continuous"><parent link="base"/><child link="shoulder"/><origin xyz="0 0 0.3"/><axis xyz="0 0 1"/></joint>
  <joint name="pitch" type="revolute"><parent link="shoulder"/><child link="elbow"/><origin xyz="0.9 0 0" rpy="0.2 0 0"/><axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
  <joint name="extension" type="prismatic"><parent link="elbow"/><child link="tool"/><origin xyz="0.7 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.8"/></joint>
</robot>`;
let model = [], frames = [], local = [], selected = '', serial = 0;
let angle = -.65, elevation = .5, zoom = 1, center = [0,0,0], fitted = false;
let inFlight = false;
const canvas = $('canvas'), ctx = canvas.getContext('2d');

function status(text, good = false) {
  $('status').textContent = text;
  $('light').style.background = good ? '#208b70' : '#bf7750';
}
function safeText(target, value) { target.textContent = value; }
function call(service, args) { return {op:'call_service', service, args, id:'frame-lab-'+(++serial)}; }
function pub(topic, msg) { return {op:'publish', topic, msg}; }
function advertise(topic, type) { return {op:'advertise', topic, type}; }
function subscriptions() {
  return [{op:'subscribe',topic:'/tf',type:'tf/tfMessage',id:'tf-'+(++serial)},
    {op:'subscribe',topic:'/xform_world',type:'xform_world/MatrixTransformArray',id:'world-'+(++serial)}];
}
async function bridge(commands, collect=.35) {
  const controller = new AbortController(), timer = setTimeout(() => controller.abort(), 6500);
  try {
    const response = await fetch('/api/bridge', {method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({commands,collect,timeout:4}),signal:controller.signal});
    const data = await response.json();
    if (!response.ok || data.error) throw Error(data.error || 'Bridge HTTP '+response.status);
    $('wire').textContent = JSON.stringify(data,null,2);
    if (data.timed_out || data.pending_ids?.length) throw Error('Runtime service response timed out: '+(data.pending_ids||[]).join(', '));
    const messages = data.messages || [];
    for (const message of messages) {
      if (message.op === 'service_response' && message.result === false) {
        // An unset description/status is displayed, while explicit writes must succeed.
        if (message.service !== '/param_server/get_param') throw Error(message.status || 'Service rejected request');
      }
      if (message.op === 'publish' && message.topic === '/tf') local = message.msg.transforms || [];
      if (message.op === 'publish' && message.topic === '/xform_world') frames = message.msg.transforms || [];
    }
    showFrames();
    return messages;
  } catch (error) {
    $('wire').textContent = error.message;
    status(error.name === 'AbortError' ? 'Bridge request timed out' : error.message);
    throw error;
  } finally { clearTimeout(timer); }
}
function answer(messages, command) {
  const result = messages.find(m => m.op==='service_response' && m.id===command.id);
  if (!result) throw Error('No service response for '+command.service);
  if (result.result !== true) throw Error(result.status || 'Parameter not set');
  return result.values;
}
async function action(fn) {
  if(inFlight) return;
  inFlight = true;
  document.querySelectorAll('button').forEach(b=>b.disabled=true);
  try { await fn(); }
  catch(error) { status(error.message); $('description').textContent=error.message; }
  finally { inFlight=false; document.querySelectorAll('button').forEach(b=>b.disabled=false); }
}
function parseDescription(xml) {
  const doc = new DOMParser().parseFromString(xml, 'application/xml');
  if(doc.querySelector('parsererror')) throw Error('URDF is not well-formed XML');
  if(doc.documentElement.localName !== 'robot') throw Error('Root element must be robot');
  const child = (el,name) => [...el.children].find(c=>c.localName===name);
  model = [...doc.documentElement.children].filter(el=>el.localName==='joint').map(el=>{
    const type = el.getAttribute('type'), limit = child(el,'limit');
    let min=Number(limit?.getAttribute('lower')??-Math.PI), max=Number(limit?.getAttribute('upper')??Math.PI);
    if(type==='continuous'){min=-2*Math.PI;max=2*Math.PI;}
    if(!Number.isFinite(min)||!Number.isFinite(max)||max<=min){min=-3;max=3;}
    return {name:el.getAttribute('name'),type,min,max,q:0,parent:child(el,'parent')?.getAttribute('link'),child:child(el,'child')?.getAttribute('link')};
  });
  $('joints').replaceChildren();
  for(const joint of model.filter(j=>j.type!=='fixed')){
    const div=document.createElement('div');div.className='joint';
    const label=document.createElement('div'), name=document.createElement('span'), number=document.createElement('input');
    name.textContent=joint.name+' · '+joint.type;number.type='number';number.step='.01';number.value='0';number.min=joint.min;number.max=joint.max;
    const slider=document.createElement('input');slider.type='range';slider.min=joint.min;slider.max=joint.max;slider.step='.001';slider.value='0';slider.setAttribute('aria-label',joint.name);
    number.setAttribute('aria-label',joint.name+' numeric value');
    slider.oninput=()=>{joint.q=Number(slider.value);number.value=joint.q.toFixed(3);};
    number.oninput=()=>{const n=Number(number.value);if(Number.isFinite(n)){joint.q=n;slider.value=String(n);}};
    label.append(name,number);div.append(label,slider);$('joints').append(div);
  }
  if(!model.some(j=>j.type!=='fixed')) $('joints').textContent='This robot has no movable joints.';
}
async function capture(commands=[]) {
  const messages=await bridge([...subscriptions(),...commands],.35);
  const current=messages.filter(m=>m.op==='publish'&&m.topic==='/xform_world');
  if(!current.length){status('Connected · no world frames received');$('frameCount').textContent='No /xform_world publication. Check accepted description and /tf.';}
  else status('Connected · '+frames.filter(f=>f.child_frame_id!=='global_frame').length+' link frames',true);
}
async function loadDescription() {
  const xml=$('urdf').value;
  parseDescription(xml);
  const command=call('/param_server/set_param',{name:'robot_description',value:xml});
  const version=answer(await bridge([command],.05),command).version;
  for(let attempt=0;attempt<8;attempt++){
    const statusCall=call('/param_server/get_param',{name:'/robot_state_publisher/description_status'});
    const messages=await bridge([statusCall],.2);
    const reply=messages.find(m=>m.id===statusCall.id&&m.op==='service_response');
    const value=reply?.values?.value;
    if(value && value.version>=version){
      $('description').textContent=JSON.stringify(value,null,2);
      if(!value.accepted) throw Error('URDF rejected: '+value.error+' (loaded version '+value.loaded_version+')');
      fitted=false;frames=[];local=[];await publishJoints();return;
    }
  }
  throw Error('Description status did not reach version '+version+' within the pickup interval');
}
function jointCommand(){
  const movable=model.filter(j=>j.type!=='fixed'), now=Date.now();
  return pub('/joint_states',{header:{stamp:{sec:Math.floor(now/1000),nanosec:(now%1000)*1000000},frame_id:''},name:movable.map(j=>j.name),position:movable.map(j=>j.q)});
}
async function publishJoints(){
  await capture([advertise('/joint_states','sensor_msgs/JointState'),jointCommand()]);
}
function poseCommand(){
  const v=Object.fromEntries(['x','y','z','roll','pitch','yaw'].map(k=>[k,Number($('pose-'+k).value)]));
  if(Object.values(v).some(x=>!Number.isFinite(x)))throw Error('Root pose must contain finite numbers');
  const [cr,cp,cy]=[v.roll,v.pitch,v.yaw].map(x=>Math.cos(x/2));
  const [sr,sp,sy]=[v.roll,v.pitch,v.yaw].map(x=>Math.sin(x/2));
  return pub('/global_pose',{position:{x:v.x,y:v.y,z:v.z},orientation:{x:sr*cp*cy-cr*sp*sy,y:cr*sp*cy+sr*cp*sy,z:cr*cp*sy-sr*sp*cy,w:cr*cp*cy+sr*sp*sy}});
}
function usefulFrames(){return frames.filter(f=>f.child_frame_id!=='global_frame' && Array.isArray(f.matrix)&&f.matrix.length===16);}
function showFrames(){
  const list=usefulFrames();
  if(!selected || !list.some(f=>f.child_frame_id===selected))selected=list[0]?.child_frame_id||'';
  $('links').replaceChildren();
  for(const frame of list){const button=document.createElement('button');button.textContent=frame.child_frame_id;button.className=frame.child_frame_id===selected?'active':'';button.onclick=()=>{selected=frame.child_frame_id;showFrames();};$('links').append(button);}
  $('frameCount').textContent=list.length+' links · '+local.length+' local edges';
  const frame=list.find(f=>f.child_frame_id===selected);
  $('selected').textContent=frame?'global_frame → '+selected:'Select a frame';
  $('matrix').replaceChildren();
  if(frame){
    for(let r=0;r<4;r++)for(let c=0;c<4;c++){const cell=document.createElement('span');cell.textContent=Number(frame.matrix[4*c+r]).toFixed(5);$('matrix').append(cell);}
    const edge=local.find(t=>t.child_frame_id===selected);
    $('edge').textContent=edge?JSON.stringify(edge,null,2):'Root frame: no incoming local /tf edge.\nStamp: '+JSON.stringify(frame.header?.stamp);
  }
  if(!fitted&&list.length){fit();fitted=true;}draw();
}
function fit(){
  const points=usefulFrames().map(f=>f.matrix.slice(12,15));
  if(!points.length)return;
  const lo=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),hi=[0,1,2].map(i=>Math.max(...points.map(p=>p[i])));
  center=lo.map((v,i)=>(v+hi[i])/2);zoom=1/Math.max(.8,Math.hypot(...hi.map((v,i)=>v-lo[i])));draw();
}
function project(p){
  const v=p.map((x,i)=>x-center[i]),ca=Math.cos(angle),sa=Math.sin(angle),ce=Math.cos(elevation),se=Math.sin(elevation);
  const horizontal=ca*v[0]-sa*v[1],depth=sa*v[0]+ca*v[1],vertical=ce*v[2]-se*depth;
  const scale=Math.min(canvas.clientWidth,canvas.clientHeight)*.68*zoom;
  return [canvas.clientWidth/2+scale*horizontal,canvas.clientHeight/2-scale*vertical,ce*depth+se*v[2]];
}
function line(a,b,color,width=1){const p=project(a),q=project(b);ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
function draw(){
  const ratio=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;
  if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=w*ratio;canvas.height=h*ratio;}
  ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
  const size=2/zoom,step=Math.pow(10,Math.floor(Math.log10(size/8)));
  for(let i=-12;i<=12;i++){const x=Math.round(center[0]/step)*step+i*step,y=Math.round(center[1]/step)*step+i*step;line([x,center[1]-size,0],[x,center[1]+size,0],'#e4ece8');line([center[0]-size,y,0],[center[0]+size,y,0],'#e4ece8');}
  const data=usefulFrames(),byName=Object.fromEntries(data.map(f=>[f.child_frame_id,f]));
  for(const edge of local){const a=byName[edge.header?.frame_id],b=byName[edge.child_frame_id];if(a&&b)line(a.matrix.slice(12,15),b.matrix.slice(12,15),'#526f68',4);}
  for(const frame of data.sort((a,b)=>project(a.matrix.slice(12,15))[2]-project(b.matrix.slice(12,15))[2])){
    const m=frame.matrix,p=m.slice(12,15),point=project(p),active=frame.child_frame_id===selected;
    if($('axes').checked)for(let c=0;c<3;c++)line(p,p.map((v,r)=>v+m[4*c+r]*.15/zoom),['#d46854','#25997d','#4e7bbd'][c],active?3:1.5);
    ctx.beginPath();ctx.arc(point[0],point[1],active?6:4,0,2*Math.PI);ctx.fillStyle=active?'#e5a643':'#12776d';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=1.5;ctx.stroke();
    ctx.font='10px Consolas,monospace';ctx.fillStyle='#47615a';ctx.fillText(frame.child_frame_id,point[0]+9,point[1]-7);
  }
  if(!data.length){ctx.fillStyle='#7b9188';ctx.font='13px Segoe UI,sans-serif';ctx.textAlign='center';ctx.fillText('Load a robot or capture the running world frames',w/2,h/2);ctx.textAlign='left';}
}
$('urdf').value=sample;
for(const [name,label] of [['x','X'],['y','Y'],['z','Z'],['roll','Roll'],['pitch','Pitch'],['yaw','Yaw']]){const cell=document.createElement('label');cell.textContent=label;const input=document.createElement('input');input.type='number';input.id='pose-'+name;input.step='.1';input.value='0';cell.append(input);$('pose').append(cell);}
$('example').onclick=()=>{$('urdf').value=sample;$('description').textContent='Example copied. Click Load robot into runtime.';};
$('load').onclick=()=>action(loadDescription);
$('fetch').onclick=()=>action(async()=>{const command=call('/param_server/get_param',{name:'robot_description'});const values=answer(await bridge([command],.05),command);if(typeof values.value!=='string')throw Error('Stored description is not a string');$('urdf').value=values.value;parseDescription(values.value);$('description').textContent='Fetched robot_description version '+values.version;await capture();});
$('refresh').onclick=()=>action(()=>capture());
$('apply').onclick=()=>action(publishJoints);
$('applyPose').onclick=()=>action(()=>capture([advertise('/global_pose','geometry_msgs/Pose'),poseCommand()]));
$('fit').onclick=fit;$('axes').onchange=draw;
let drag=null;
canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{if(!drag)return;angle+=(e.clientX-drag[0])*.01;elevation=Math.max(-1.4,Math.min(1.4,elevation+(e.clientY-drag[1])*.01));drag=[e.clientX,e.clientY];draw();};
canvas.onpointerup=()=>drag=null;canvas.onpointercancel=()=>drag=null;
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.002,Math.min(100,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
new ResizeObserver(draw).observe(canvas);
draw();
