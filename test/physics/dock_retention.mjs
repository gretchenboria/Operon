import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
import fs from "node:fs"; import path from "node:path";
const ROOT=REPO;
globalThis.fetch=async u=>new Response(fs.readFileSync(path.join(ROOT,"public",String(u)),"utf8"),{status:200});
const { initPhysics } = await import(`${ROOT}/public/mujoco-physics.js`);
const FRAME=1/60;
const SLOT=[0.1,0.105224,0.072622];
const NZ=[0,0.42289,0.90618];
let fail=0; const ok=(c,m)=>{console.log(`${c?"  PASS":"  FAIL"}  ${m}`);if(!c)fail++;};
const p=await initPhysics({vendorBase:`${ROOT}/node_modules/mujoco/`,modelUrl:"/models/operon_station0.xml",rigs:[6]});
const B=p.arms[6]; B.setStation(1);
const settle=s=>{const n=Math.round(s/FRAME);for(let i=0;i<n;i++)B.advance(FRAME);};
// park the arm well clear so only the dock is in play
B._data.ctrl[1]=-1.2; B._data.ctrl[2]=1.6; settle(1.0);

// Drop the cell into the cradle: axis along the dock's local y, released a
// few mm above the pocket floor.
const q=B._cellFree.qpos, v=B._cellFree.qvel;
const drop=[SLOT[0]+NZ[0]*0.006, SLOT[1]+NZ[1]*0.006, SLOT[2]+NZ[2]*0.006];
q[0]=drop[0]; q[1]=drop[1]; q[2]=drop[2];
// quat whose z column is the dock's local y = (0, 0.906, -0.423)
q[3]=0.537175; q[4]=-0.843471; q[5]=0; q[6]=0;
for(let i=0;i<6;i++) v[i]=0;
settle(0.05);
console.log(`  released at z=${B.read().cellP[2].toFixed(4)}`);
settle(2.5);
let r=B.read();
let d=Math.hypot(r.cellP[0]-SLOT[0],r.cellP[1]-SLOT[1],r.cellP[2]-SLOT[2]);
console.log(`  2.5 s later: cell [${r.cellP.map(x=>x.toFixed(4))}], ${(d*1000).toFixed(1)} mm from slot, |v|=${r.cellVel.toFixed(4)}, contacts=${r.contacts}`);
ok(d<0.030,`cradle RETAINS the cell under gravity (${(d*1000).toFixed(1)} mm from slot)`);
ok(r.cellP[2]>0.045,`cell stays at dock height (z=${r.cellP[2].toFixed(3)}), not on the table`);
ok(r.cellVel<0.01,"cell settles at rest in the cradle");
settle(5.0);
r=B.read(); d=Math.hypot(r.cellP[0]-SLOT[0],r.cellP[1]-SLOT[1],r.cellP[2]-SLOT[2]);
console.log(`  7.5 s total: ${(d*1000).toFixed(1)} mm from slot, z=${r.cellP[2].toFixed(4)}`);
ok(d<0.032,"still docked after 7.5 s — not creeping out");
console.log(`\n${fail===0?"CRADLE RETENTION PASSED":fail+" FAILED"}`);
B.dispose(); process.exitCode=fail===0?0:1;
