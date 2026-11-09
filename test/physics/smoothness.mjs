import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
// Smoothness over REAL trajectories, not a single step:
//   A) the full scripted pick & place (multi-segment eased tweens)
//   B) continuous live wrist-tilt steering
//   C) a hard step command (worst case)
import fs from "node:fs"; import path from "node:path";
const ROOT=REPO;
globalThis.fetch=async u=>new Response(fs.readFileSync(path.join(ROOT,"public",String(u)),"utf8"),{status:200});
const { initPhysics } = await import(`${ROOT}/public/mujoco-physics.js`);
const { ik6, fk6, poseR } = await import(`${ROOT}/public/demos/arms3d.js`);
const FRAME=1/60;
const ease=k=>k<0.5?4*k*k*k:1-Math.pow(-2*k+2,3)/2;
const smooth=(c,t,k)=>c.map((a,i)=>{let d=t[i]-a;while(d>Math.PI)d-=2*Math.PI;while(d<-Math.PI)d+=2*Math.PI;return a+d*k;});
const lerp=(a,b,e)=>a.map((v,i)=>v+(b[i]-v)*e);

// ── A: the scripted cycle's command stream (same waypoints & durations) ──
function pickPlaceCmds(){
  const zG=0.0135+0.011, cp=[0.15,-0.075];
  const wp=[
    {p:[cp[0],cp[1],zG+0.075], yaw:-25, ms:520},
    {p:[cp[0],cp[1],zG],       yaw:-25, ms:520},
    {p:[cp[0],cp[1],zG],       yaw:-25, ms:600},   // dwell: arrival + clamp
    {p:[cp[0],cp[1],zG+0.075], yaw:-25, ms:560},
    {p:[0.15,0.075,zG+0.085],  yaw:25,  ms:720},
    {p:[0.15,0.075,zG],        yaw:25,  ms:560},
    {p:[0.15,0.075,zG+0.085],  yaw:25,  ms:480},
  ];
  const out=[]; let cur=[0.15,-0.075,0.12], curYaw=-25;
  for(const w of wp){
    const n=Math.round(w.ms/1000/FRAME);
    for(let i=1;i<=n;i++){ const e=ease(i/n); out.push({p:lerp(cur,w.p,e), yaw:curYaw+(w.yaw-curYaw)*e}); }
    cur=w.p.slice(); curYaw=w.yaw;
  }
  return out;
}
// ── B: continuous wrist tilt (what the Watch actually produces) ──
function tiltCmds(){
  const out=[]; const N=Math.round(6.0/FRAME);
  let x=0.15,y=0,z=0.12;
  for(let i=0;i<N;i++){
    const t=i*FRAME;
    const tgtY=0.122*Math.sin(2*Math.PI*0.35*t)+0.004*Math.sin(2*Math.PI*6.1*t);  // tilt + wrist tremor
    const tgtZ=0.080+0.070*Math.sin(2*Math.PI*0.22*t+1.1)+0.003*Math.sin(2*Math.PI*8.3*t);
    const kp=Math.min(1,FRAME*14.5);                 // the studio's tilt tracker
    if(Math.abs(tgtY-y)>0.001) y+=(tgtY-y)*kp;
    if(Math.abs(tgtZ-z)>0.001) z+=(tgtZ-z)*kp;
    out.push({p:[x,y,z], yaw:(y/0.122)*30*0.40});
  }
  return out;
}
function stepCmds(){
  const out=[];
  for(let i=0;i<90;i++) out.push(i<15?{p:[0.15,-0.075,0.12],yaw:-25}:{p:[0.15,0.075,0.075],yaw:-25});
  return out;
}

function metrics(tips){
  const v=[],a=[],j=[];
  for(let i=1;i<tips.length;i++) v.push(tips[i].map((x,k)=>(x-tips[i-1][k])/FRAME));
  for(let i=1;i<v.length;i++) a.push(v[i].map((x,k)=>(x-v[i-1][k])/FRAME));
  for(let i=1;i<a.length;i++) j.push(a[i].map((x,k)=>(x-a[i-1][k])/FRAME));
  const mag=A=>A.map(w=>Math.hypot(...w));
  const rms=A=>Math.sqrt(mag(A).reduce((s,x)=>s+x*x,0)/A.length);
  return {pkV:Math.max(...mag(v)), pkA:Math.max(...mag(a)), pkJ:Math.max(...mag(j)), rmsJ:rms(j)};
}

const p=await initPhysics({vendorBase:`${ROOT}/node_modules/mujoco/`,modelUrl:"/models/operon_station0.xml",rigs:[6]});
const S=p.arms[6];

function runKinematic(cmds){
  let q=ik6(cmds[0].p,poseR(cmds[0].yaw,0,0)).q; const tips=[];
  for(const c of cmds){ const raw=ik6(c.p,poseR(c.yaw,0,0),q).q; q=smooth(q,raw,0.94); tips.push(fk6(q).tip); }
  return tips;
}
function runPhysics(cmds){
  S.teleportTo(ik6(cmds[0].p,poseR(cmds[0].yaw,0,0)).q,1);
  let q=ik6(cmds[0].p,poseR(cmds[0].yaw,0,0)).q; const tips=[];
  for(const c of cmds){ const raw=ik6(c.p,poseR(c.yaw,0,0),q).q; q=smooth(q,raw,0.94); S.setArmTarget(q); S.advance(FRAME); tips.push(S.read().tip); }
  return tips;
}

console.log("TCP smoothness. Jerk is the standard metric: lower = less snap.\n");
console.log("trajectory                 metric        analytic IK      MuJoCo      change");
let allGood=true;
for(const [name,cmds] of [["full pick & place",pickPlaceCmds()],["live wrist tilt (6 s)",tiltCmds()],["hard step command",stepCmds()]]){
  const k=metrics(runKinematic(cmds)), m=metrics(runPhysics(cmds));
  const row=(lbl,a,b,u)=>{
    const pct=((b/a-1)*100);
    console.log(`  ${name.padEnd(24)} ${lbl.padEnd(12)} ${(a.toFixed(a<100?2:0)+' '+u).padStart(14)} ${(b.toFixed(b<100?2:0)+' '+u).padStart(12)}   ${pct>0?'+':''}${pct.toFixed(0)}%`);
  };
  row("peak accel",k.pkA,m.pkA,"m/s2");
  row("peak jerk",k.pkJ,m.pkJ,"m/s3");
  row("rms jerk",k.rmsJ,m.rmsJ,"m/s3");
  if(m.rmsJ>=k.rmsJ) allGood=false;
  console.log("");
}
console.log(allGood?"RMS jerk reduced on EVERY trajectory type":"a trajectory type got rougher");
S.dispose(); process.exitCode=allGood?0:1;
