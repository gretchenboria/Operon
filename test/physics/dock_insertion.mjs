import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
import fs from "node:fs"; import path from "node:path";
const ROOT=REPO;
globalThis.fetch=async u=>new Response(fs.readFileSync(path.join(ROOT,"public",String(u)),"utf8"),{status:200});
const { initPhysics } = await import(`${ROOT}/public/mujoco-physics.js`);
const { ik6, poseR } = await import(`${ROOT}/public/demos/arms3d.js`);
const FRAME=1/60, G={h0:0.090,L1:0.100,L2:0.100,L3:0.060};
const nrm=a=>{const l=Math.hypot(...a);return[a[0]/l,a[1]/l,a[2]/l];};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const fromZX=(z,x)=>{z=nrm(z);x=nrm(x);const y=cross(z,x);return[x[0],y[0],z[0],x[1],y[1],z[1],x[2],y[2],z[2]];};
const DOCK_R=fromZX([0,-0.42,-0.9],[1,0,0]);   // tool +z points INTO the dock
const NZ=[0,0.42289,0.90618];                 // dock OUTWARD normal
const SLOT=[0.1,0.105224,0.072622];
const along=r=>[SLOT[0]+NZ[0]*r,SLOT[1]+NZ[1]*r,SLOT[2]+NZ[2]*r];
const reach=(p3,R)=>{const z=[R[2],R[5],R[8]];const w=[p3[0]-G.L3*z[0],p3[1]-G.L3*z[1],p3[2]-G.L3*z[2]];
  return Math.hypot(w[0],w[1],w[2]-G.h0);};
// slerp-ish orientation blend via orthonormalised lerp
const blend=(A,Bm,f)=>{const M=A.map((v,i)=>v*(1-f)+Bm[i]*f);
  const z=nrm([M[2],M[5],M[8]]); let x=[M[0],M[3],M[6]];
  const d=x[0]*z[0]+x[1]*z[1]+x[2]*z[2]; x=nrm([x[0]-d*z[0],x[1]-d*z[1],x[2]-d*z[2]]);
  return fromZX(z,x);};
let fail=0; const ok=(c,m)=>{console.log(`${c?"  PASS":"  FAIL"}  ${m}`);if(!c)fail++;};
const p=await initPhysics({vendorBase:`${ROOT}/node_modules/mujoco/`,modelUrl:"/models/operon_station0.xml",rigs:[6]});
const B=p.arms[6]; B.setStation(1);
const settle=s=>{const n=Math.round(s/FRAME);for(let i=0;i<n;i++)B.advance(FRAME);};
const goto=(p3,R,grip,s)=>{const q=ik6(p3,R,B.read().q).q;B.setArmTarget(q);if(grip!==undefined)B.setGripTarget(grip);settle(s);};

const PICK_R=poseR(-25,0,0);
B.teleportTo(ik6([0.15,-0.075,0.15],PICK_R).q,1);
B.placeCell(0.15,-0.075,-25); settle(0.6);
const cp=B.read().cellP.slice(), zG=cp[2]+0.011;
goto([cp[0],cp[1],zG+0.07],PICK_R,1,0.6);
goto([cp[0],cp[1],zG],PICK_R,1,0.7);
goto([cp[0],cp[1],zG],PICK_R,0,0.5);
ok(B.read().held,"grasped at the pick pad");
goto([cp[0],cp[1],zG+0.09],PICK_R,0,0.6);

const STAND=0.034;
// Carry in three clean phases instead of blending position and orientation at
// once: lift, re-orient IN PLACE at a comfortably reachable point, then
// translate to the dock standoff with the orientation already fixed. Blending
// both together walked the arm through poses it could not track and it came
// up 50 mm short every time.
const SAFE=[0.130,0.020,0.150];
console.log("    phase                 TCPerr(mm)  held  cell-TCP(mm)");
const report=(lbl,tgt)=>{const r=B.read();
  const e=Math.hypot(r.tip[0]-tgt[0],r.tip[1]-tgt[1],r.tip[2]-tgt[2]);
  const d=Math.hypot(r.cellP[0]-r.tip[0],r.cellP[1]-r.tip[1],r.cellP[2]-r.tip[2]);
  console.log(`    ${lbl.padEnd(22)} ${(e*1000).toFixed(1).padStart(8)}   ${r.held?"yes":"NO "}  ${(d*1000).toFixed(1).padStart(10)}`);};
goto(SAFE,PICK_R,0,0.8); report("lift to safe point",SAFE);
const N=10;
for(let k=1;k<=N;k++){ const f=k/N; goto(SAFE,blend(PICK_R,DOCK_R,f),0,0.22); }
report("re-orient in place",SAFE);
const stand=along(STAND);
// Route OVER the dock: traverse at a constant safe height (the cradle tops
// out near z=0.071) and only then descend onto the standoff. A straight line
// from the safe point to the standoff clips the cradle and strips the cell
// out of the jaws.
const over=[stand[0],stand[1],0.155];
for(let k=1;k<=10;k++){ const f=k/10;
  goto([SAFE[0]+(over[0]-SAFE[0])*f, SAFE[1]+(over[1]-SAFE[1])*f, SAFE[2]+(over[2]-SAFE[2])*f], DOCK_R, 0, 0.26); }
report("traverse above dock",over);
for(let k=1;k<=8;k++){ const f=k/8;
  goto([over[0], over[1], over[2]+(stand[2]-over[2])*f], DOCK_R, 0, 0.26); }
report("descend to standoff",stand);
ok(B.read().held,"grasp survived the re-orientation carry to the dock");
console.log("    lower  cmdTCP->          actualTCP->        err(mm) cell-slot(mm) ncon");
for(let k=1;k<=8;k++){ const f=k/8; const tp=along(STAND*(1-f)+0.012*f);
  goto(tp,DOCK_R,0,0.26); const r=B.read();
  const e=Math.hypot(r.tip[0]-tp[0],r.tip[1]-tp[1],r.tip[2]-tp[2]);
  const ds=Math.hypot(r.cellP[0]-SLOT[0],r.cellP[1]-SLOT[1],r.cellP[2]-SLOT[2]);
  console.log(`    ${String(k).padStart(5)}  [${tp.map(v=>v.toFixed(3))}] [${[...r.tip].map(v=>v.toFixed(3))}] ${(e*1000).toFixed(1).padStart(6)} ${(ds*1000).toFixed(1).padStart(10)} ${String(r.contacts).padStart(4)}`);
}
const low=B.read();
console.log(`  lowered: ${(Math.hypot(low.cellP[0]-SLOT[0],low.cellP[1]-SLOT[1],low.cellP[2]-SLOT[2])*1000).toFixed(1)} mm from slot, held=${low.held}, contacts=${low.contacts}`);
ok(low.held && Math.hypot(low.cellP[0]-SLOT[0],low.cellP[1]-SLOT[1],low.cellP[2]-SLOT[2])<0.030,"cell brought into the cradle still held");
B.setGripTarget(0.015/0.022); settle(0.5);
for(let k=1;k<=5;k++) goto(along(0.012+0.010*k),DOCK_R,undefined,0.26);
B.setGripTarget(1.0); settle(1.5);
const fin=B.read();
const d=Math.hypot(fin.cellP[0]-SLOT[0],fin.cellP[1]-SLOT[1],fin.cellP[2]-SLOT[2]);
console.log(`  after release+withdraw: ${(d*1000).toFixed(1)} mm from slot, z=${fin.cellP[2].toFixed(4)}, held=${fin.held}, |v|=${fin.cellVel.toFixed(4)}`);
ok(!fin.held,"jaws released");
ok(d<0.030,`cell SEATED in the cradle (${(d*1000).toFixed(1)} mm)`);
settle(3.0);
const l2=B.read(); const d2=Math.hypot(l2.cellP[0]-SLOT[0],l2.cellP[1]-SLOT[1],l2.cellP[2]-SLOT[2]);
ok(d2<0.032,`stays docked 3 s later (${(d2*1000).toFixed(1)} mm)`);
console.log(`\n${fail===0?"FULL PHYSICAL DOCK INSERTION PASSED":fail+" FAILED"}`);
B.dispose(); process.exitCode=fail===0?0:1;
