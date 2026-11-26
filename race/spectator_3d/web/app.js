import * as T from 'three';
import {OBJLoader} from 'three/addons/loaders/OBJLoader.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {ColladaLoader} from 'three/addons/loaders/ColladaLoader.js';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id), error=e=>{$('error').style.display='block';$('error').textContent=String(e)};
const scene=new T.Scene();scene.background=new T.Color(0x8299b3);const camera=new T.PerspectiveCamera(50,innerWidth/innerHeight,.03,2500);camera.up.set(0,0,1);const renderer=new T.WebGLRenderer({antialias:true,powerPreference:'high-performance'});renderer.setSize(innerWidth,innerHeight);$('view').append(renderer.domElement);renderer.setPixelRatio(1);scene.add(new T.HemisphereLight(0xffffff,0x777777,2.1));const sun=new T.DirectionalLight(0xffffff,2);sun.position.set(-40,30,100);scene.add(sun);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=true;orbit.enabled=false;
const loader=new OBJLoader(), stlLoader=new STLLoader(), daeLoader=new ColladaLoader(), texLoader=new T.TextureLoader();
const signalLenses=new Map(),signalObjects=new Map();let signalRoot=null,hudLensOrder=[1,2,3,4,5];let signalPhase="waiting",signalCount=0,fastestLap=null;
let cardKey=null,cardTimer=null;
function clockIcon(){return '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/></svg>'}
function lapFormat(t){const ms=Math.round(t*1000);return String(Math.floor(ms/60000)).padStart(2,'0')+':'+String(Math.floor(ms/1000)%60).padStart(2,'0')+'.'+String(ms%1000).padStart(3,'0')}
function showSignal(sig,sent){
 const phase=sig?.phase||"waiting",count=phase==="countdown"?Math.max(0,Math.min(5,sig.count||0)):0;
 if(phase!==signalPhase||count!==signalCount){signalPhase=phase;signalCount=count;if(signalRoot)signalRoot.visible=phase==="countdown";for(const [i,mat] of signalLenses){const on=i<=count;mat.color.setRGB(...(on?[.12,.006,.008]:[.015,.005,.006]));mat.emissive.setRGB(...(on?[1,.008,.008]:[0,0,0]));mat.emissiveIntensity=on?2:0}for(let i=1;i<=5;i++)$("lamp"+i).classList.toggle("lit",hudLensOrder[i-1]<=count)}
 const panel=$("startSignal"),fastest=$("fastestLap"),lamps=$("signalLamps"),label=$("signalLabel");
 const record=phase==='started'&&sig.start_cleared&&fastestLap&&Number.isFinite(fastestLap.time)?fastestLap:null;
 const next=phase==='started'&&sig.start_cleared?(record?'fast:'+record.team+':'+record.time:'empty'):'signal';
 function paint(){panel.classList.toggle('fast-mode',!!record);lamps.hidden=!!record;fastest.hidden=!record;label.hidden=!!record;
 if(record){$("fastestTeam").textContent=record.team.toUpperCase();$("fastestTime").textContent=lapFormat(record.time)}
 else label.textContent=phase==='started'?'START':phase==='error'?'신호 오류 · 출발 대기':phase==='countdown'?'START SIGNAL':'출발 대기';}
 if(next===cardKey){if(next==='signal')paint();return}
 const old=cardKey;cardKey=next;clearTimeout(cardTimer);
 if(next==='empty'){panel.classList.add('slide-away');return}
 if(old===null||next==='signal'){paint();panel.classList.remove('slide-away');return}
 panel.classList.add('slide-away');
 cardTimer=setTimeout(()=>{paint();requestAnimationFrame(()=>panel.classList.remove('slide-away'))},old==='empty'?0:400);

}
function transform(obj,p){obj.position.set(...p.slice(0,3));obj.rotation.set(p[3],p[4],p[5],'ZYX')}
async function makeModel(m){const root=new T.Group();transform(root,m.pose);for(const l of m.links){const link=new T.Group();transform(link,l.pose);root.add(link);for(const v of l.visuals){let ob;const material=new T.MeshStandardMaterial({color:new T.Color(...v.color),roughness:.7,side:T.DoubleSide,transparent:(v.opacity??1)<1,opacity:v.opacity??1});if(v.kind==='mesh'){if(v.texture){const tex=await texLoader.loadAsync('/'+v.texture);tex.colorSpace=T.SRGBColorSpace;material.map=tex;}const path='/'+v.url;if(path.toLowerCase().endsWith('.stl')){ob=new T.Mesh(await stlLoader.loadAsync(path),material)}else if(path.toLowerCase().endsWith('.dae')){ob=(await daeLoader.loadAsync(path)).scene;ob.traverse(x=>{if(x.isMesh&&!v.texture&&!x.material)x.material=material})}else{ob=await loader.loadAsync(path);ob.traverse(x=>{if(x.isMesh)x.material=material})}ob.scale.set(...v.scale)}else if(v.kind==='box'){ob=new T.Mesh(new T.BoxGeometry(...v.size),material)}else if(v.kind==='sphere'){ob=new T.Mesh(new T.SphereGeometry(v.radius,24,16),material)}else{const g=new T.CylinderGeometry(v.radius,v.radius,v.length,16);ob=new T.Mesh(g,material)}if(m.name==="urrc_start_signal_active"){const match=/^active_red_lens_([1-5])$/.exec(v.name||"");if(match){const id=Number(match[1]);signalLenses.set(id,material);signalObjects.set(id,ob)}}transform(ob,v.pose);link.add(ob)}}return root}
let stage="test",lapCount=3,runningCount=0,mapRoute=[],mapHits=[];
let manifest,carTemplates=new Map(),carLoading=new Map(),mode='free',selected=null,zoom=1,rows=[],frames=[],cars=new Map(),lastArrival=0,viewReady=false,lastSelected=null,camReset=true,mid=new T.Vector3(),span=100,lapLength=0;
async function init(){const r=await fetch('/manifest.json');if(!r.ok)throw Error('맵 리소스 없음');manifest=await r.json();const staticRoot=new T.Group();for(const m of manifest.static)staticRoot.add(await makeModel(m));scene.add(staticRoot);signalRoot=await makeModel(manifest.signal_active);scene.add(signalRoot);signalRoot.updateMatrixWorld(true);const yaw=manifest.signal_yaw,right=new T.Vector3(Math.sin(yaw),-Math.cos(yaw),0);hudLensOrder=[...signalObjects.entries()].sort((a,b)=>a[1].getWorldPosition(new T.Vector3()).dot(right)-b[1].getWorldPosition(new T.Vector3()).dot(right)).map(x=>x[0]);if(hudLensOrder.length!==5)throw Error('Gazebo 신호등 5개 위치 확인 실패');signalRoot.visible=false;for(const [team,car] of Object.entries(manifest.cars||{})){carLoading.set(team,makeModel(car).then(model=>{carTemplates.set(team,model);return model}).catch(err=>{console.error('차량 모델 로딩 실패',team,err);return null}))}await Promise.all(carLoading.values());const bounds=new T.Box3().setFromObject(staticRoot);bounds.getCenter(mid);const sz=bounds.getSize(new T.Vector3());span=Math.max(sz.x,sz.y);viewReady=true;free();fetch("/track.json").then(r=>r.json()).then(r=>{mapRoute=r;drawMap()}).catch(error);connect();}
function free(){mode='free';orbit.enabled=true;orbit.target.copy(mid);camera.position.copy(mid).add(new T.Vector3(0,-span*.45,span*.8));orbit.update()}
function choose(team){if(!rows.some(r=>r.team===team))return;selected=team;mode='follow';orbit.enabled=false;camReset=true;updateHud()}
// Server sample clock, not packet-arrival clock. Each vehicle keeps only
// distinct simulator samples. Network clock offset is estimated from recent
// low-delay packets; a buffered playback clock absorbs jitter.
const tracks=new Map();let clockOffset=null,offsetSamples=[],bufferMs=140,targetBuffer=140,playAt=null;
let received=0,sourceHz=0,simRate=0,jitter=0,slowFrames=0,frameWorst=0,underruns=0,lastPacketAt=0;
// Finish celebrations run only in the browser. They never publish or command Gazebo.
const finishedSeen=new Map();let snapshotInitialized=false,celebrationBatch=[];
const fxCanvas=document.getElementById('finishFireworks');
const fxContext=fxCanvas.getContext('2d');
let sparks=[],shockwaves=[],flashes=[],fxRunning=false,fxLast=0;
const celebrationColors=['#ffd966','#ff5252','#55caff','#90f7ad','#d5a6ff','#fff3ab'];
function fxSize(){const ratio=Math.min(devicePixelRatio||1,1.5);fxCanvas.width=Math.ceil(innerWidth*ratio);fxCanvas.height=Math.ceil(innerHeight*ratio);fxContext.setTransform(ratio,0,0,ratio,0,0)}
fxSize();addEventListener('resize',fxSize);
function burst(cx,cy,size=1){
 // A bright core flash, an expanding shock ring and two layers of flying sparks.
 flashes.push({x:cx,y:cy,age:0,life:.22,radius:24*size});
 shockwaves.push({x:cx,y:cy,age:0,life:.53,radius:100*size});
 const color=celebrationColors[Math.floor(Math.random()*celebrationColors.length)];
 for(let k=0;k<100;k++){
  const angle=Math.PI*2*(k/100)+Math.random()*.16;
  const speed=(k%3===0?190:105)+Math.random()*170;
  sparks.push({x:cx,y:cy,px:cx,py:cy,vx:Math.cos(angle)*speed*size,vy:Math.sin(angle)*speed*size,
   age:0,life:.85+Math.random()*1.05,color:k%5===0?'#fffbe6':k%3===0?color:celebrationColors[(k+Math.floor(cx))%celebrationColors.length],
   radius:1.1+Math.random()*2.5,drag:1.4+Math.random()*.65});
 }
}
function animateFireworks(now){
 if(!fxRunning)return;
 const dt=Math.min((now-(fxLast||now))/1000,.05);fxLast=now;
 fxContext.clearRect(0,0,innerWidth,innerHeight);
 sparks=sparks.filter(p=>{
  p.age+=dt;if(p.age>=p.life)return false;
  p.px=p.x;p.py=p.y;p.vx*=Math.exp(-p.drag*dt);p.vy=p.vy*Math.exp(-p.drag*dt)+120*dt;
  p.x+=p.vx*dt;p.y+=p.vy*dt;
  const fade=Math.pow(1-p.age/p.life,1.6);
  fxContext.globalAlpha=fade;
  fxContext.strokeStyle=p.color;fxContext.lineWidth=p.radius;
  fxContext.beginPath();fxContext.moveTo(p.px,p.py);fxContext.lineTo(p.x,p.y);fxContext.stroke();
  fxContext.fillStyle=p.color;fxContext.beginPath();fxContext.arc(p.x,p.y,p.radius*.72,0,Math.PI*2);fxContext.fill();
  return true;
 });
 shockwaves=shockwaves.filter(w=>{
  w.age+=dt;if(w.age>=w.life)return false;
  const t=w.age/w.life;
  fxContext.globalAlpha=(1-t)*.7;fxContext.strokeStyle='#fff0b4';fxContext.lineWidth=2.8*(1-t)+.4;
  fxContext.beginPath();fxContext.arc(w.x,w.y,w.radius*t,0,Math.PI*2);fxContext.stroke();return true;
 });
 flashes=flashes.filter(f=>{
  f.age+=dt;if(f.age>=f.life)return false;
  const t=f.age/f.life;
  const glow=fxContext.createRadialGradient(f.x,f.y,0,f.x,f.y,f.radius*(1+t));
  glow.addColorStop(0,'rgba(255,255,235,.95)');glow.addColorStop(.2,'rgba(255,222,120,.55)');glow.addColorStop(1,'rgba(255,200,90,0)');
  fxContext.globalAlpha=(1-t)*.95;fxContext.fillStyle=glow;
  fxContext.beginPath();fxContext.arc(f.x,f.y,f.radius*(1+t),0,Math.PI*2);fxContext.fill();return true;
 });
 fxContext.globalAlpha=1;
 if(sparks.length||shockwaves.length||flashes.length)requestAnimationFrame(animateFireworks);
 else {fxRunning=false;fxLast=0;fxContext.clearRect(0,0,innerWidth,innerHeight)}
}
function finishCelebration(team){
 const focused=selected===team;
 // Three staggered 'POP' waves, from opposite edges of the unobstructed track view.
 const patterns=[
  [[.17,.27,1.12],[.83,.29,1.12]],
  [[.11,.47,.90],[.89,.48,.90]],
  [[.26,.19,.78],[.74,.19,.78]]
 ];
 patterns.forEach((wave,i)=>setTimeout(()=>{
  for(const [x,y,size] of wave)burst(innerWidth*x,innerHeight*y,size*(focused?1.13:1));
  if(!fxRunning){fxRunning=true;fxLast=0;requestAnimationFrame(animateFireworks)}
 },i*430));
 const notice=document.createElement('div');notice.className='finish-notice';
 const title=document.createElement('strong');title.textContent=team.toUpperCase()+' · FINISHED';
 const subtitle=document.createElement('small');subtitle.textContent=focused?'관전 차량 완주':'완주 차량';
 notice.append(title,subtitle);
 document.getElementById('finishNotices').append(notice);
 setTimeout(()=>{notice.classList.add('hide');setTimeout(()=>notice.remove(),350)},3500);
}
function updateFinishEvents(incoming){
 // On first packet, existing finished states are baselines, not new finishes.
 for(const r of incoming){
  const before=finishedSeen.get(r.team);
  if(snapshotInitialized&&before===false&&r.finished===true)finishCelebration(r.team);
  finishedSeen.set(r.team,Boolean(r.finished));
 }
 snapshotInitialized=true;
}

function connect(){
 const key=new URLSearchParams(location.search).get('key');
 const socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws?key=${encodeURIComponent(key||'')}`);
 socket.onmessage=e=>{
  const data=JSON.parse(e.data),now=performance.now();rows=data.teams;stage=data.stage||"test";lapCount=data.lap_count||3;runningCount=data.running_count??rows.length;
  if(selected&&rows.find(r=>r.team===selected)?.retired){
    const next=rows.find(r=>!r.retired&&!r.finished);
    if(next)choose(next.team);
  }lapLength=data.lap_length;fastestLap=data.fastest_lap;lastArrival=now;showSignal(data.signal,data.sent_ms);updateFinishEvents(rows);
  if(Number.isFinite(data.sent_ms)){
   offsetSamples.push({at:now,value:now-data.sent_ms});while(offsetSamples.length&&offsetSamples[0].at<now-10000)offsetSamples.shift();
   const floor=Math.min(...offsetSamples.map(x=>x.value));clockOffset=clockOffset===null?floor:clockOffset+(floor-clockOffset)*.04;
   const delay=Math.max(0,now-data.sent_ms-clockOffset);jitter=jitter*.92+delay*.08;targetBuffer=T.MathUtils.clamp(120+3*jitter,120,220);
  }
  for(const r of rows){if(!Number.isFinite(r.sample_ms))continue;let tr=tracks.get(r.team);if(!tr){tr=[];tracks.set(r.team,tr)}const last=tr.at(-1);
   if(last&&r.t<last.t){tr.length=0;playAt=null} else if(last&&r.t===last.t)continue;
   const prev=tr.at(-1);if(prev&&r.sample_ms<=prev.sample_ms)continue;
   if(prev&&r.team===selected){const wall=(r.sample_ms-prev.sample_ms)/1000;const sim=r.t-prev.t;if(wall>0){sourceHz=sourceHz?sourceHz*.9+.1/wall:1/wall;simRate=simRate?simRate*.9+.1*sim/wall:sim/wall}}
   tr.push(r);received++;while(tr.length>180)tr.shift();
  }
  if(selected&&!rows.some(r=>r.team===selected)){if(rows.length)choose(rows[0].team);else{selected=null;free()}}
 };
 socket.onclose=()=>{fastestLap=null;showSignal({phase:"waiting",count:0},null);tracks.clear();rows=[];lastArrival=0;clockOffset=null;offsetSamples=[];playAt=null;setTimeout(connect,1200)};
 socket.onerror=()=>socket.close();
}
const stageNames={test:'테스트',qualifying:'예선',semi_a:'본선 A',semi_b:'본선 B',final:'결승'};
function drawMap(){
 const cv=$('map'),c=cv.getContext('2d'),w=cv.width,h=cv.height; c.clearRect(0,0,w,h);mapHits=[];if(!mapRoute.length)return;
 const xs=mapRoute.map(p=>p[0]),ys=mapRoute.map(p=>p[1]),minX=Math.min(...xs),minY=Math.min(...ys),dx=Math.max(...xs)-minX,dy=Math.max(...ys)-minY,k=Math.min((w-150)/Math.max(dx,1),(h-65)/Math.max(dy,1));
 const pt=p=>[(w-dx*k)/2+(p[0]-minX)*k,(h+dy*k)/2-(p[1]-minY)*k];
 c.beginPath();mapRoute.forEach((p,i)=>{const [x,y]=pt(p);i?c.lineTo(x,y):c.moveTo(x,y)});c.closePath();c.strokeStyle='#8996a5';c.lineWidth=6;c.lineJoin='round';c.stroke();
 const labels=[];c.font='bold 20px system-ui';for(const r of rows){const [x,y]=pt(cars.get(r.team)?.position.toArray()||r.p);const name=r.team.toUpperCase(),tw=c.measureText(name).width;let lx=Math.min(w-tw-5,Math.max(5,x+12)),ly=Math.min(h-6,Math.max(23,y-8));for(let n=0;n<15;n++){if(!labels.some(a=>lx<a.x+a.w+4&&lx+tw+4>a.x&&Math.abs(ly-a.y)<23))break;ly+=24;if(ly>h-6)ly=23}labels.push({x:lx,y:ly,w:tw});c.strokeStyle='#aab7ca';c.lineWidth=1.5;c.beginPath();c.moveTo(x,y);c.lineTo(lx,ly-7);c.stroke();c.fillStyle=r.team===selected?'#4da3ff':'#ffffff';c.beginPath();c.arc(x,y,r.team===selected?8:5,0,Math.PI*2);c.fill();c.strokeStyle='#10151d';c.lineWidth=4;c.strokeText(name,lx,ly);c.fillText(name,lx,ly);mapHits.push({x,y,lx,ly,tw,team:r.team})}
}
$('map').onclick=e=>{const b=e.currentTarget.getBoundingClientRect(),x=(e.clientX-b.left)*e.currentTarget.width/b.width,y=(e.clientY-b.top)*e.currentTarget.height/b.height;const hit=mapHits.find(p=>Math.hypot(p.x-x,p.y-y)<18||(x>=p.lx&&x<=p.lx+p.tw&&y>=p.ly-23&&y<=p.ly+4));if(hit)choose(hit.team)};
function updateHud(){
 const el=$('teams'),podiumRows=rows.filter(r=>r.rank<=3);
 const signature=podiumRows.map(r=>r.team+':'+r.rank+':'+r.finished+':'+r.status).join(',')+'|'+selected;
 if(el.dataset.sig!==signature){
  const oldPositions=new Map([...el.querySelectorAll('button.team')].map(b=>[b.dataset.team,b.getBoundingClientRect()]));
  const existing=new Map([...el.querySelectorAll('button.team')].map(b=>[b.dataset.team,b]));
  el.dataset.sig=signature;
  const children=[];
  for(let rank=1;rank<=3;rank++){
   const r=rows.find(x=>x.rank===rank);
   if(!r){const placeholder=document.createElement('div');placeholder.className='podium-placeholder';placeholder.textContent=`${rank}등  —`;children.push(placeholder);continue}
   let b=existing.get(r.team);
   if(!b){
    b=document.createElement('button');b.className='team';b.dataset.team=r.team;
    for(const [cls,val] of [['name',''],['fast-clock',''],['arrow',''],['gap','']]){const x=document.createElement('span');x.className=cls;x.textContent=val;b.append(x)}
    b.onclick=()=>choose(r.team);
   }
   b.className='team'+(r.team===selected?' on':'')+(r.status==='DNF'||r.retired?' retired':'');
   b.querySelector('.name').textContent=`${rank}등  ${r.team.toUpperCase()}`;
   children.push(b);
  }
  el.replaceChildren(...children);
  // FLIP: move each existing team visually from its previous slot into
  // its new rank. DOM order remains the real ranking for accessibility.
  if(!window.matchMedia('(prefers-reduced-motion: reduce)').matches){
   for(const b of el.querySelectorAll('button.team')){
    const old=oldPositions.get(b.dataset.team),current=b.getBoundingClientRect();
    if(old&&Math.abs(old.top-current.top)>2){
     b.animate([{transform:`translateY(${old.top-current.top}px)`,zIndex:2},{transform:'translateY(0)',zIndex:1}],
       {duration:680,easing:'cubic-bezier(.22,1,.36,1)'});
    }else if(!old){b.animate([{opacity:0},{opacity:1}],{duration:350,easing:'ease-out'})}
   }
  }
 }
 for(const b of el.querySelectorAll('button')){
  const r=rows.find(r=>r.team===b.dataset.team);if(!r)continue;
  const arrow=b.querySelector('.arrow'),clock=b.querySelector('.fast-clock');
  clock.hidden=!r.is_fastest;clock.title='FASTEST LAP';if(r.is_fastest&&!clock.firstChild)clock.innerHTML=clockIcon();
  arrow.textContent=r.arrow>0?'▲':r.arrow<0?'▼':'';arrow.className='arrow '+(r.arrow>0?'green':r.arrow<0?'red':'');
  b.querySelector('.gap').textContent=(r.status==='DNF'||r.retired)?'RETIRED':r.finished?'FINISH':r.rank===1?'LEADER':r.ahead_gap==null?'—':(r.ahead_gap_estimated?'≈ +':'+')+r.ahead_gap.toFixed(3)+' s';
 }
 $('activeCount').textContent=runningCount+'대 주행';$('stage').textContent=stageNames[stage]||stage;$('lapTotal').textContent=lapCount;
 const r=rows.find(r=>r.team===selected);$('target').textContent=r?`P${r.rank}/${rows.length} · ${r.team.toUpperCase()}`:'FOLLOW —';$('lap').textContent=r?(r.finished?'FINISH':Math.min(lapCount,r.laps+1)):'—';$('speed').textContent=r?(r.speed*3.6).toFixed(1):'—';$('ms').textContent=r?r.speed.toFixed(2):'—';$('peak').textContent=r?(r.peak*3.6).toFixed(1)+' km/h':'—';$('time').textContent=r&&r.lap_time!=null?r.lap_time.toFixed(3)+' s':'—';
 drawMap();
}
let prev=performance.now(),fps=0,count=0,mark=prev,hudAt=0;const up=new T.Vector3(0,0,1),forward=new T.Vector3(),desired=new T.Vector3(),look=new T.Vector3(),smoothLook=new T.Vector3();
const tmpPos=new T.Vector3(),tmpQuat=new T.Quaternion();
function draw(now){requestAnimationFrame(draw);const frameMs=now-prev,dt=Math.min(.1,frameMs/1000);prev=now;count++;if(frameMs>25)slowFrames++;frameWorst=Math.max(frameWorst,frameMs);
 if(now-mark>1000){fps=Math.round(count*1000/(now-mark));count=0;mark=now;$('status').textContent=(lastArrival&&now-lastArrival<1500?'LIVE':'수신 대기')+' · '+fps+' FPS';$('status').title=`Pose ${sourceHz.toFixed(1)} Hz | RTF ${simRate.toFixed(2)} | buffer ${Math.round(bufferMs)} ms | jitter ${jitter.toFixed(1)} ms | slow frames ${slowFrames} | max frame ${frameWorst.toFixed(1)} ms | underruns ${underruns}`;const diag=$('diagnostics');if(diag)diag.textContent=$('status').title;slowFrames=0;frameWorst=0;underruns=0}
 if(!viewReady)return;
 bufferMs+=(targetBuffer-bufferMs)*Math.min(1,dt*.8);
 const wanted=clockOffset===null?null:now-clockOffset-bufferMs;
 if(wanted!==null){if(playAt===null||Math.abs(wanted-playAt)>1000)playAt=wanted;else playAt+=Math.max(0,frameMs+T.MathUtils.clamp((wanted-playAt-frameMs)*.15,-frameMs*.2,frameMs*.2))}
 const active=new Set(rows.map(r=>r.team));for(const [id,obj] of cars){if(!active.has(id)){scene.remove(obj);cars.delete(id);tracks.delete(id)}}
 for(const r of rows){let ob=cars.get(r.team);if(!ob){const source=carTemplates.get(r.team);if(!source)continue;ob=source.clone(true);cars.set(r.team,ob);scene.add(ob)}
 const tr=tracks.get(r.team)||[];if(playAt!==null)while(tr.length>2&&tr[1].sample_ms<=playAt)tr.shift();
 const x=tr[0]||r,y=tr[1]||x;const spanMs=y.sample_ms-x.sample_ms;const alpha=playAt!==null&&spanMs>0?T.MathUtils.clamp((playAt-x.sample_ms)/spanMs,0,1):0;
 ob.position.fromArray(x.p).lerp(tmpPos.fromArray(y.p),alpha);ob.quaternion.fromArray(x.q).normalize().slerp(tmpQuat.fromArray(y.q).normalize(),alpha);
 // Only short extrapolation on genuine buffer underrun; never continue stale motion indefinitely.
 if(playAt!==null&&spanMs>0&&playAt>y.sample_ms){if(r.team===selected)underruns++;const extra=Math.min(50,playAt-y.sample_ms);if(y.speed>.05&&spanMs<250){tmpPos.fromArray(y.p).sub(new T.Vector3().fromArray(x.p));ob.position.addScaledVector(tmpPos,Math.min(.8,extra/spanMs))}}
 }
 const ob=cars.get(selected);if(mode!=='free'&&ob){forward.set(1,0,0).applyQuaternion(ob.quaternion);forward.z=0;forward.normalize();if(forward.lengthSq()<.01)forward.set(1,0,0);const top=mode==='top';desired.copy(ob.position).addScaledVector(forward,top?-1:-8).addScaledVector(up,top?20:6);look.copy(ob.position).addScaledVector(forward,top?1:5.5);if(camReset||lastSelected!==selected){camera.position.copy(desired);smoothLook.copy(look);camReset=false;lastSelected=selected}else{const f=1-Math.exp(-dt*10);camera.position.lerp(desired,f);smoothLook.lerp(look,f)}const fov=T.MathUtils.clamp(50/zoom,20,90);if(camera.fov!==fov){camera.fov=fov;camera.updateProjectionMatrix()}camera.lookAt(smoothLook)}else if(mode==='free'){orbit.update()}
 if(now-hudAt>100){updateHud();hudAt=now}renderer.render(scene,camera)}
$('diag').onclick=()=>{const d=$('diagnostics');d.hidden=!d.hidden};$('follow').onclick=()=>{if(selected)choose(selected);else if(rows.length)choose(rows[0].team)};$('top').onclick=()=>{if(!selected&&rows.length)selected=rows[0].team;if(selected){mode='top';orbit.enabled=false;camReset=true}};$('free').onclick=free;$('reset').onclick=()=>{zoom=1;camReset=true;if(mode==='free'){camera.fov=50;camera.updateProjectionMatrix();free()}};function adjust(f){if(mode==='free'){camera.position.sub(orbit.target).multiplyScalar(1/f).add(orbit.target)}else zoom=T.MathUtils.clamp(zoom*f,.6,2.5)}$('plus').onclick=()=>adjust(1.2);$('minus').onclick=()=>adjust(1/1.2);renderer.domElement.addEventListener('wheel',e=>{if(mode!=='free'){e.preventDefault();adjust(Math.exp(-e.deltaY*.001))}},{passive:false});$('quality').onchange=()=>{renderer.setPixelRatio(Number($('quality').value));renderer.setSize(innerWidth,innerHeight)};$('hud').onclick=()=>document.body.classList.toggle('clean');$('full').onclick=()=>document.fullscreenElement?document.exitFullscreen():document.documentElement.requestFullscreen();addEventListener('resize',()=>{renderer.setSize(innerWidth,innerHeight);camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix()});init().catch(error);requestAnimationFrame(draw);
