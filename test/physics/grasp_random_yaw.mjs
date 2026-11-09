import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
import fs from "node:fs"; import path from "node:path";
const ROOT=REPO;
globalThis.fetch = async (u) => new Response(fs.readFileSync(path.join(ROOT,"public",String(u)),"utf8"),{status:200});
const { initPhysics } = await import(`${ROOT}/public/mujoco-physics.js`);
const { ik6, poseR } = await import(`${ROOT}/public/demos/arms3d.js`);
const D2R=Math.PI/180, PHYS_GRASP_H=0.011, FRAME=1/60;

function graspYaw(r, fb){ const ax=r.cellR[2], ay=r.cellR[5];
  if(!Number.isFinite(ax)||Math.hypot(ax,ay)<1e-6) return fb;
  let y=Math.atan2(-ax,ay)/D2R; while(y>90)y-=180; while(y<-90)y+=180; return y; }

const p = await initPhysics({ vendorBase:`${ROOT}/node_modules/mujoco/`, modelUrl:"/models/operon_station0.xml", rigs:[6] });
const B=p.arms[6];
const settle=s=>{const n=Math.round(s/FRAME);for(let i=0;i<n;i++)B.advance(FRAME);};
const goto=(p3,yaw,grip,s)=>{const q=ik6(p3,poseR(yaw,0,0),B.read().q).q;B.setArmTarget(q);if(grip!==undefined)B.setGripTarget(grip);settle(s);};

let pass=0, total=0;
console.log(" cellYaw   x,y (mm)      approach   jawGap   padL/padR     lifted   held");
for (const spec of [[0.150,-0.075,0],[0.150,-0.075,-25],[0.150,-0.075,35],[0.120,-0.050,-60],[0.180,-0.100,70],[0.135,-0.095,15],[0.165,-0.040,-80],[0.150,-0.075,89]]) {
  const [x,y,yaw]=spec; total++;
  B.teleportTo(ik6([0.15,-0.075,0.14],poseR(-25,0,0)).q,1);
  B.placeCell(x,y,yaw); settle(0.6);
  const r0=B.read(); const cp=r0.cellP.slice(); const gy=graspYaw(r0,-25); const zG=cp[2]+PHYS_GRASP_H;
  goto([cp[0],cp[1],zG+0.07],gy,1,0.7);
  goto([cp[0],cp[1],zG],gy,1,0.7);
  goto([cp[0],cp[1],zG],gy,0,0.5);
  const rc=B.read(); const z0=rc.cellP[2];
  goto([cp[0],cp[1],zG+0.085],gy,0,0.9);
  const rl=B.read(); const lift=(rl.cellP[2]-z0)*1000;
  const good = lift>55 && rl.held;
  if(good) pass++;
  console.log(` ${String(yaw).padStart(5)}째  ${(x*1000).toFixed(0)},${(y*1000).toFixed(0)}`.padEnd(24)+
    `${gy.toFixed(1).padStart(7)}째  ${rc.jawGapMm.toFixed(1).padStart(6)}mm  ${rc.padLoadN.map(v=>v.toFixed(1)).join("/").padStart(9)}N  ${lift.toFixed(0).padStart(6)}mm   ${good?"OK":"FAIL"}`);
}
console.log(`\n${pass}/${total} randomised cell poses grasped and lifted`);
B.dispose();
process.exitCode = pass===total?0:1;
