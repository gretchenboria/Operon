import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const APP = process.env.OPERON_URL || "http://127.0.0.1:8841";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
import WebSocket from "ws";
import { spawn } from "node:child_process";
import { rmSync } from "node:fs";
const PROF="/tmp/cr-prof-"+process.pid;
const CDP=9371;
const chrome=spawn("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
 ["--headless=new","--disable-gpu","--no-sandbox","--no-first-run","--use-gl=angle","--use-angle=swiftshader",
  "--enable-unsafe-swiftshader",`--remote-debugging-port=${CDP}`,"--remote-allow-origins=*",
  "--window-size=1500,950",`--user-data-dir=${PROF}`,"about:blank"],{stdio:"ignore"});
process.on("exit",()=>{try{chrome.kill("SIGKILL")}catch{};try{rmSync(PROF,{recursive:true,force:true})}catch{}});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
let t=null;
for(let i=0;i<60;i++){try{const l=await (await fetch(`http://127.0.0.1:${CDP}/json/list`)).json();t=l.find(x=>x.type==="page");if(t?.webSocketDebuggerUrl)break;}catch{} await sleep(250);}
const ws=new WebSocket(t.webSocketDebuggerUrl,{perMessageDeflate:false});
await new Promise(r=>ws.once("open",r));
let id=0;const pend=new Map();
ws.on("message",raw=>{const m=JSON.parse(raw.toString());if(m.id&&pend.has(m.id)){pend.get(m.id)(m);pend.delete(m.id);}});
const send=(me,pa={})=>new Promise(r=>{const i=++id;pend.set(i,m=>r(m.result));ws.send(JSON.stringify({id:i,method:me,params:pa}));});
const ev=async e=>(await send("Runtime.evaluate",{expression:e,returnByValue:true,awaitPromise:true}))?.result?.value;
await send("Runtime.enable");await send("Page.enable");
await send("Page.navigate",{url:`${APP}/teleop-studio.html`});
for(let i=0;i<90;i++){await sleep(400);const b=await ev(`document.getElementById("phys-badge")?.textContent||""`);if(b&&!/booting/i.test(b))break;}
let fail=0;const ok=(c,m)=>{console.log(`${c?"  PASS":"  FAIL"}  ${m}`);if(!c)fail++;};

console.log("Does the RENDERED scene equal the solver's state?\n");
const probeRaw = await ev(`(()=>{ try { return JSON.stringify(window.__operonRenderProbe ? window.__operonRenderProbe() : "NO PROBE"); } catch(e){ return "THREW: "+e.message; } })()`);
console.log("  probe sample:", String(probeRaw).slice(0,400));
await ev(`document.getElementById("btn-pick-place").click()`);
const errsP=[],errsJ=[];
for(let i=0;i<26;i++){
  await sleep(300);
  const pr=await ev(`JSON.stringify(window.__operonRenderProbe?.()||{})`);
  if(!pr) continue;
  const d=JSON.parse(pr);
  for(const dof of ["4","6"]){
    const e=d[dof]; if(!e||!e.solved) continue;
    // rendered world pos includes the rig's display offset in Y; remove it
    const r3=e.renderedRobotFrame;
    errsP.push(Math.hypot(r3[0]-e.solved[0], r3[1]-e.solved[1], r3[2]-e.solved[2]));
    if(e.jawOpenRendered!=null && e.jawOpenSolved!=null) errsJ.push(Math.abs(e.jawOpenRendered-e.jawOpenSolved));
  }
}
const mx=a=>Math.max(...a), mean=a=>a.reduce((x,y)=>x+y,0)/a.length;
console.log(`  samples: ${errsP.length} cell-pose comparisons over a full pick & place`);
console.log(`  rendered-vs-solved cell position: mean ${(mean(errsP)*1e6).toFixed(2)} um, worst ${(mx(errsP)*1e6).toFixed(2)} um`);
console.log(`  rendered-vs-solved jaw aperture : worst ${mx(errsJ).toExponential(2)}`);
ok(errsP.length > 10, `collected enough samples to be meaningful (${errsP.length})`);
ok(errsP.length > 0 && mx(errsP) < 1e-6, "the cell is DRAWN exactly where the solver put it (sub-micron)");
ok(errsJ.length > 0 && mx(errsJ) < 1e-9, "the jaws are DRAWN at exactly the solved finger position");
console.log(`\n${fail===0?"RENDER FIDELITY PASSED":fail+" FAILED"}`);
ws.close();process.exit(fail===0?0:1);
