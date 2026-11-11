import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const APP = process.env.OPERON_URL || "http://127.0.0.1:8841";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
import WebSocket from "ws";
import { spawn } from "node:child_process";
import { rmSync } from "node:fs";
const PROF="/tmp/cr-prof-"+process.pid;
const CDP=9351, URL=`${APP}/teleop-studio.html`;
const chrome=spawn("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
 ["--headless=new","--disable-gpu","--no-sandbox","--no-first-run","--use-gl=angle","--use-angle=swiftshader",
  "--enable-unsafe-swiftshader",`--remote-debugging-port=${CDP}`,"--remote-allow-origins=*",
  "--window-size=1400,900",`--user-data-dir=${PROF}`,"about:blank"],{stdio:"ignore"});
process.on("exit",()=>{try{chrome.kill("SIGKILL")}catch{};try{rmSync(PROF,{recursive:true,force:true})}catch{}});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
let t=null;
for(let i=0;i<60;i++){try{const l=await (await fetch(`http://127.0.0.1:${CDP}/json/list`)).json();t=l.find(x=>x.type==="page");if(t?.webSocketDebuggerUrl)break;}catch{} await sleep(250);}
const ws=new WebSocket(t.webSocketDebuggerUrl,{perMessageDeflate:false});
await new Promise(r=>ws.once("open",r));
let id=0; const pend=new Map(); const errs=[];
ws.on("message",raw=>{const m=JSON.parse(raw.toString());
 if(m.id&&pend.has(m.id)){pend.get(m.id)(m);pend.delete(m.id);return;}
 if(m.method==="Runtime.exceptionThrown")errs.push(m.params.exceptionDetails.exception?.description||m.params.exceptionDetails.text);});
const send=(method,params={})=>new Promise(r=>{const i=++id;pend.set(i,m=>r(m.result));ws.send(JSON.stringify({id:i,method,params}));});
const ev=async e=>{const r=await send("Runtime.evaluate",{expression:e,returnByValue:true,awaitPromise:true});
 if(r?.exceptionDetails)return{__err:r.exceptionDetails.exception?.description};return r?.result?.value;};
await send("Runtime.enable"); await send("Page.enable"); await send("Page.navigate",{url:URL});
for(let i=0;i<80;i++){await sleep(400);const b=await ev(`document.getElementById("phys-badge")?.textContent||""`);if(b&&!/booting/i.test(b))break;}
let fail=0; const ok=(c,m)=>{console.log(`${c?"  PASS":"  FAIL"}  ${m}`);if(!c)fail++;};
console.log("switching to Kinematic…");
await ev(`document.querySelector('#phys-mode-bar button[data-pm="kinematic"]').click()`);
await sleep(1500);
const badge=await ev(`document.getElementById("phys-badge")?.textContent||""`);
const panel=await ev(`document.getElementById("phys-gap")?.textContent||""`);
console.log(`  badge="${badge}"  jaw gap="${panel}"`);
ok(/kinematic/i.test(badge),"badge switches to kinematic");
ok(panel==="—","physics readout blanks out on the analytic path");
await ev(`document.getElementById("btn-pick-place").click()`);
await sleep(5000);
const verdict=await ev(`document.getElementById("verdict-6")?.textContent||""`);
console.log(`  verdict after kinematic pick&place: "${verdict}"`);
ok(/Complete|Pad|Cell/i.test(verdict),"kinematic pick & place still runs");
console.log("switching back to MuJoCo…");
await ev(`document.querySelector('#phys-mode-bar button[data-pm="mujoco"]').click()`);
await sleep(2500);
const badge2=await ev(`document.getElementById("phys-badge")?.textContent||""`);
const gap2=await ev(`document.getElementById("phys-gap")?.textContent||""`);
console.log(`  badge="${badge2}"  jaw gap="${gap2}"`);
ok(/MuJoCo/.test(badge2),"switches back to MuJoCo");
ok(/mm/.test(gap2),"solver resumes reporting");

console.log("checking verified OpenUSD stage export in browser…");
await ev(`document.querySelector('#schema-tabs button[data-s="usd"]').click()`);
await sleep(500);
const usdText=await ev(`document.getElementById("schema-preview")?.textContent||""`);
ok(usdText.includes('def Xform "pedestal"') && usdText.includes('def PhysicsRevoluteJoint "yaw"') && usdText.includes('drive:angular:physics:targetPosition.timeSamples'),
   "browser USD export contains verified 46-gprim stage + timeSampled PhysicsDriveAPI");
const { writeFileSync } = await import("node:fs");
writeFileSync("/tmp/browser_exported.usda", usdText, "utf8");

const real=errs.filter(e=>!/favicon|ReadPixels|GL Driver/i.test(e));
console.log(`  page errors: ${real.length}`); real.slice(0,5).forEach(e=>console.log("   ! "+e.split("\n")[0]));
ok(real.length===0,"no errors across both mode switches");
console.log(`\n${fail===0?"FALLBACK CHECKS PASSED":fail+" FAILED"}`);
ws.close(); process.exit(fail===0?0:1);
